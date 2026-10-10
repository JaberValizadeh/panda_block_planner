"""The perceive-plan-act-verify loop.

The run r in the model is a path through E. This does not replay one; it
produces one. Every iteration it reads the world, abstracts it into a state,
searches E for the shortest route to the goal, executes the *first* action of
that plan, and looks again.

Re-planning after every single action rather than executing a plan end to end
is the whole design, and it is there because gamma is not a function. In
simulation, executing a0 in s0 can yield s1, or leave the world in s0 because
the grasp closed on nothing, or put the block somewhere no state predicted.
Searching E costs microseconds; a wrong assumption about where a block is
costs the run. So the plan is advice, recomputed each time, and the world is
the authority.

The world and the arm are constructor arguments rather than things this class
builds, so the loop can be tested against fakes. That matters more here than
it usually does: the interesting behaviour is what happens when an action
*fails*, and failures are exactly what a working simulator will not produce on
demand.
"""

import rospy

from . import domain, geometry, planner

# An action that leaves the world in the state it was already in is not
# progress. Three of those in a row on the same action and something is wrong
# that another attempt will not fix - a grasp that keeps missing, a block
# wedged against the table edge - so stop rather than grind.
MAX_RETRIES = 3
# Generous: the Sussman scenario needs 6, an inverted tower 6, the five-block
# reversal 12, and retries consume budget too. It exists to bound a livelock,
# not to be tuned.
MAX_ACTIONS = 40


class Task(object):
    def __init__(self, scenario, world, arm):
        self.scenario = scenario
        self.world = world
        self.arm = arm
        self.trace_states = []
        self.trace_actions = []
        self.trace_ok = []

    # -- helpers -----------------------------------------------------------
    def observe(self):
        # Settle first: abstracting a block that is still in flight reads it
        # onto a level it is only passing through. See WorldModel.settle.
        self.world.settle(held=self.arm.held)
        obs = self.world.perceive(held=self.arm.held)
        for problem in obs.problems:
            rospy.logwarn("perception: %s", problem)
        return obs

    def goal_footprint(self, obs):
        """Where the goal tower is going, so a buffer slot avoids it.

        Only atom goals name the place. A callable goal ("any three-block
        tower") deliberately does not, which is the point of it - there is
        nothing to avoid because the planner has not committed yet.
        """
        if callable(self.scenario.goal):
            return []
        out = []
        for atom in self.scenario.goal:
            if atom[0] == "on" and atom[2] != domain.TABLE:
                base = geometry.base_of(obs.state, atom[2])
                # A base that is currently in the hand has no footprint yet -
                # its xy is where the gripper is, which says nothing about
                # where the tower will stand. Same reason a callable goal
                # contributes nothing here.
                if base in obs.poses and base != obs.held:
                    out.append(obs.xy(base))
        return out

    def choose_slot(self, obs, block):
        """An empty (x, y) for a block that has to go down on the table.

        `block` is excluded because it is the one being placed. The *held*
        block is excluded for a subtler reason: perception reports an xy for
        it too, and that xy is wherever the hand has carried it - directly
        over the tower being built, as often as not. Counted as occupied, it
        rules out the nearest slot, and the buffer slides outboard to one the
        arm can barely reach over. That is not hypothetical: it is what put
        the five-block tower at x = 0.54 instead of 0.42, and what made a
        six-high tower unplannable altogether, since 0.54 is past the reach
        ceiling for one.
        """
        occupied = [obs.xy(b) for b in obs.poses
                    if b != block and b != obs.held]
        return geometry.free_slot(occupied, avoid=self.goal_footprint(obs))

    def plan_from(self, obs):
        # The buffer slot is not chosen yet when planning, so the filter gets
        # the slot the executor *would* pick - close enough, and the loop
        # re-plans against reality straight after anyway.
        try:
            hint = self.choose_slot(obs, None)
        except RuntimeError:
            hint = geometry.TABLE_CENTER[:2]
        feasible = geometry.reach_filter(obs.table_xy(), hint)
        result = planner.plan(obs.state, self.scenario.goal, feasible=feasible)
        if result is None:
            rospy.logerr("No plan from %s", domain.format_state(obs.state))
            rospy.logerr("Reason: %s", planner.explain_failure(
                obs.state, self.scenario.goal, feasible))
        return result

    # -- the loop ----------------------------------------------------------
    def run(self):
        rospy.loginfo("Scenario '%s': %s", self.scenario.name,
                      self.scenario.description)
        rospy.loginfo("Goal: %s", self.scenario.goal_text)

        self.arm.add_table()
        self.arm.go_home()
        self.arm.open_gripper()

        retries = 0
        last_action = None

        for _ in range(MAX_ACTIONS):
            obs = self.observe()
            if not self.trace_states:
                self.trace_states.append(obs.state)
            rospy.loginfo("state: %s", domain.format_state(obs.state))

            fatal = [p for p in obs.problems if "fallen off" in p]
            if fatal:
                raise RuntimeError("unrecoverable: " + "; ".join(fatal))

            if planner.satisfies(obs.state, self.scenario.goal):
                rospy.loginfo("Goal satisfied.")
                self.report(obs)
                return

            result = self.plan_from(obs)
            if result is None:
                raise RuntimeError("no feasible plan from the current state")

            action = result.actions[0]
            rospy.loginfo("plan (%d actions): %s", len(result),
                          ", ".join(str(a) for a in result.actions))
            rospy.loginfo("executing %s", action.verbose())

            slot = None
            if action.kind == "stack" and action.support == domain.TABLE:
                slot = self.choose_slot(obs, action.block)

            expected = domain.apply_action(obs.state, action)
            target = self.arm.target_of(action, obs, slot)

            self.arm.sync_scene(obs)
            self.arm.execute(action, obs, slot)

            # observe(), not world.perceive(): this is the verification step,
            # so its perception problems are the ones most worth hearing about
            # - the raw call swallowed them and left the mismatch below with
            # nothing to explain it.
            after = self.observe()
            # Every attempt goes into the trace, including the ones that did
            # not work. Recording only the successes would produce a tidy run
            # that nothing actually performed.
            self.trace_actions.append(action)
            self.trace_states.append(after.state)
            self.trace_ok.append(after.state == expected)

            if after.state == expected:
                rospy.loginfo("-> %s", domain.format_state(after.state))
                retries = 0
                last_action = None
                if target is not None:
                    self.check_placement(action.block, target, after)
                continue

            # gamma did not do what the model said it would.
            rospy.logwarn("%s did not have its modelled effect", action)
            rospy.logwarn("  expected %s", domain.format_state(expected))
            rospy.logwarn("  observed %s", domain.format_state(after.state))
            retries = retries + 1 if action == last_action else 1
            last_action = action
            if retries >= MAX_RETRIES:
                raise RuntimeError("%s failed %d times in a row - stopping"
                                   % (action, retries))
            rospy.logwarn("re-planning from what is actually there "
                          "(attempt %d/%d)", retries, MAX_RETRIES)

        raise RuntimeError("gave up after %d actions" % MAX_ACTIONS)

    def check_placement(self, block, target, obs):
        """Symbolically right is not the same as physically right.

        The abstraction is happy as soon as the block is on the correct
        support; it cannot see a block hanging half off the one below it.
        This is the only check that can.
        """
        error = geometry.distance(obs.poses[block], target)
        rospy.loginfo("  %s landed %.0f mm from where it was aimed",
                      block, error * 1000.0)
        if error > geometry.PLACE_TOLERANCE:
            rospy.logwarn("  ... which is past PLACE_TOLERANCE (%.0f mm)",
                          geometry.PLACE_TOLERANCE * 1000.0)

    def report(self, obs):
        """Print the run that actually happened, in the model's notation."""
        result = planner.Plan(self.trace_states, self.trace_actions,
                              self.trace_ok)
        rospy.loginfo("The run that was executed:\n%s", result.format_run())
        for index, action in result.failures():
            rospy.logwarn("  a%d (%s) had to be redone", index, action)
        rospy.loginfo("Final block positions (Gazebo ground truth):")
        for block in sorted(obs.poses):
            rospy.loginfo("  block_%s at (%+.4f, %+.4f, %.4f)",
                          block, *obs.poses[block])
