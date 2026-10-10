"""The perceive-plan-act-verify loop, driven against a fake world.

The loop's whole reason for existing is what it does when gamma misbehaves,
and a working simulator will not misbehave on demand - an injected
disturbance races the task and usually lands in the wrong window. So the
world and the arm are faked here, and the faults are scripted: a grasp that
drops its block, an action that never takes effect at all.

Everything below models gamma exactly as domain.py defines it, except where a
fault says otherwise. That is the point: the loop must cope with the world
disagreeing with the model, without the model itself being wrong.
"""

import unittest

from panda_block_planner import domain, geometry, planner, scenarios, task
from panda_block_planner.world_model import Observation


class FakeWorld(object):
    """Holds the 'physical' truth: a state, plus where the towers stand."""

    def __init__(self, state, table_xy):
        self.state = state
        self.table_xy = dict(table_xy)

    def poses(self):
        out = {}
        for block in domain.blocks_in(self.state):
            if domain.holding(block) in self.state:
                # In the hand: somewhere above the table, exact value unused.
                out[block] = (0.45, 0.0, 0.60)
                continue
            base = geometry.base_of(self.state, block)
            x, y = self.table_xy[base]
            out[block] = (x, y, geometry.block_z(
                domain.height_of(self.state, block)))
        return out

    def settle(self, held=None, timeout=None):
        # Nothing is in flight in the fake: FakeArm applies each action
        # instantaneously, so the world is always already at rest.
        return True

    def perceive(self, held=None):
        return Observation(self.state, self.poses(), [])


class FakeArm(object):
    """Applies actions to a FakeWorld, honouring any scheduled faults.

    `faults` maps an action's repr to a fault name, consumed on first use:
      'drop'   - the block ends up on the table instead of where it was aimed
      'nothing'- the action has no effect at all (a grasp that closed on air)
    """

    def __init__(self, world, faults=None):
        self.world = world
        self.faults = dict(faults or {})
        self.held = None
        self.performed = []
        self.homed = 0

    # Motion-free stand-ins for the real executor's setup calls.
    def add_table(self):
        pass

    def go_home(self):
        self.homed += 1

    def open_gripper(self):
        pass

    def sync_scene(self, observation):
        pass

    def target_of(self, action, obs, slot=None):
        if action.kind == "pickup":
            return None
        if action.support == domain.TABLE:
            return (slot[0], slot[1], geometry.BLOCK_Z)
        x, y, sz = obs.poses[action.support]
        return (x, y, geometry.place_z_on(sz))

    def execute(self, action, obs, slot=None):
        self.performed.append(action)
        fault = self.faults.pop(repr(action), None)

        if fault == "nothing":
            return

        if action.kind == "pickup":
            self.world.state = domain.apply_action(self.world.state, action)
            self.held = action.block
            return

        if fault == "drop":
            # The grasp let go early: the block lands flat on the table,
            # which is a perfectly legal state - just not the intended one.
            put_down = domain.stack(action.block, domain.TABLE)
            self.world.state = domain.apply_action(self.world.state, put_down)
            self.world.table_xy[action.block] = (0.42, -0.28)
        else:
            self.world.state = domain.apply_action(self.world.state, action)
            if action.support == domain.TABLE:
                self.world.table_xy[action.block] = tuple(slot)
        self.held = None


def make(scenario_name="model", faults=None):
    scenario = scenarios.SCENARIOS[scenario_name]
    world = FakeWorld(scenario.initial, scenario.table_xy)
    arm = FakeArm(world, faults)
    return task.Task(scenario, world, arm), world, arm


class TestNominal(unittest.TestCase):
    def test_reaches_the_goal(self):
        job, world, arm = make()
        job.run()
        self.assertTrue(job.trace_actions)
        self.assertEqual([str(a) for a in job.trace_actions],
                         ["pickup(B)", "stack(B,C)", "pickup(A)", "stack(A,B)"])

    def test_executes_exactly_the_planned_actions(self):
        job, world, arm = make()
        job.run()
        self.assertEqual(arm.performed, job.trace_actions)

    def test_trace_is_a_valid_run(self):
        job, world, arm = make()
        job.run()
        for state, action, nxt in zip(job.trace_states, job.trace_actions,
                                      job.trace_states[1:]):
            self.assertEqual(domain.apply_action(state, action), nxt)

    def test_already_solved_does_nothing(self):
        job, world, arm = make()
        world.state = domain.state_from_towers([["C", "B", "A"]])
        world.table_xy = {"C": (0.45, 0.0)}
        job.run()
        self.assertEqual(arm.performed, [])


class TestFiveBlocks(unittest.TestCase):
    """The long one, end to end: twelve actions and every block moved."""

    def test_builds_the_whole_tower(self):
        job, world, arm = make("five_blocks")
        job.run()
        for atom in scenarios.GOAL_EDCBA:
            self.assertIn(atom, world.state, domain.format_state(world.state))
        self.assertEqual(len(arm.performed), 12)

    def test_it_fits_inside_the_action_budget_with_room_for_retries(self):
        self.assertGreater(task.MAX_ACTIONS, 12 * task.MAX_RETRIES)

    def test_the_buffer_never_lands_under_the_goal_tower(self):
        """B is put down mid-run; the tower's base must not be put on top of it."""
        job, world, arm = make("five_blocks")
        job.run()
        self.assertEqual(len(set(world.table_xy.values())),
                         len(world.table_xy),
                         "two blocks were put in the same place: %s"
                         % world.table_xy)

    def test_a_dropped_block_halfway_up_is_recovered(self):
        """stack(C,D) lets go early, four blocks into a five-block tower."""
        job, world, arm = make("five_blocks",
                               faults={repr(domain.stack("C", "D")): "drop"})
        job.run()
        performed = [str(a) for a in arm.performed]
        self.assertEqual(performed.count("stack(C,D)"), 2, performed)
        for atom in scenarios.GOAL_EDCBA:
            self.assertIn(atom, world.state)


class TestSixBlocks(unittest.TestCase):
    """The tallest one, end to end against the fake world."""

    def test_builds_the_six_high_tower(self):
        job, world, arm = make("six_blocks")
        job.run()
        for atom in scenarios.GOAL_FEDCBA:
            self.assertIn(atom, world.state, domain.format_state(world.state))
        self.assertEqual(len(arm.performed), 12)

    def test_only_one_block_is_ever_put_on_the_table(self):
        """F starts the tower; nothing else needs the table as a buffer."""
        job, world, arm = make("six_blocks")
        job.run()
        putdowns = [a for a in arm.performed
                    if a.kind == "stack" and a.support == domain.TABLE]
        self.assertEqual([str(a) for a in putdowns], ["stack(F,T)"])

    def test_a_drop_near_the_top_is_recovered(self):
        job, world, arm = make("six_blocks",
                               faults={repr(domain.stack("B", "C")): "drop"})
        job.run()
        self.assertEqual([str(a) for a in arm.performed].count("stack(B,C)"), 2)
        for atom in scenarios.GOAL_FEDCBA:
            self.assertIn(atom, world.state)


class TestBufferSlotChoice(unittest.TestCase):
    """The held block is not standing on the table, wherever perception puts it.

    It is in the gripper, and the xy reported for it is where the hand is -
    over the tower being built, more often than not. Counting it as occupied
    space rules out the nearest slot and pushes the buffer outboard, which for
    a six-high tower is the difference between a plan and no plan.
    """

    def obs_with_a_block_in_hand(self):
        scenario = scenarios.SCENARIOS["six_blocks"]
        world = FakeWorld(scenario.initial, scenario.table_xy)
        arm = FakeArm(world)
        job = task.Task(scenario, world, arm)
        arm.execute(domain.pickup("F", "E"), world.perceive())
        return job, world.perceive(held=arm.held)

    def test_the_held_block_does_not_occupy_a_slot(self):
        job, obs = self.obs_with_a_block_in_hand()
        self.assertEqual(obs.held, "F")
        self.assertEqual(job.choose_slot(obs, "F"), (min(geometry.SLOT_X), 0.0))

    def test_a_held_goal_base_has_no_footprint_to_avoid(self):
        job, obs = self.obs_with_a_block_in_hand()
        # F is the goal tower's base and F is in the hand, so where the tower
        # will stand is not known yet - nothing to steer the buffer away from.
        for xy in job.goal_footprint(obs):
            self.assertNotEqual(xy, obs.xy("F"))

    def test_the_six_high_tower_stays_plannable_all_the_way_through(self):
        """The regression this guards: it failed at step 1, not step 0."""
        job, world, arm = make("six_blocks")
        job.run()
        self.assertEqual(len(arm.performed), 12)


class TestRecovery(unittest.TestCase):
    def test_a_dropped_block_is_picked_back_up(self):
        """The failure from panda_block_stack's README, handled this time.

        stack(B,C) lets go early and B ends up on the table. Nothing about
        the arm's own report would reveal that - only the observation does.
        """
        job, world, arm = make(faults={repr(domain.stack("B", "C")): "drop"})
        job.run()

        performed = [str(a) for a in arm.performed]
        self.assertEqual(performed.count("stack(B,C)"), 2,
                         "should have retried the transfer: %s" % performed)
        self.assertEqual(performed.count("pickup(B)"), 2,
                         "and re-grasped B off the table first: %s" % performed)
        # It still finishes, and the trace records only what actually held.
        self.assertTrue(domain.on("A", "B") in world.state)
        self.assertTrue(domain.on("B", "C") in world.state)

    def test_the_failed_action_stays_in_the_trace_and_is_flagged(self):
        """The reported run must be the one that really happened.

        Dropping the failed attempt would produce a tidy sequence that
        nothing performed; keeping it unflagged would claim an edge of E that
        does not exist. So it is kept, and marked.
        """
        job, world, arm = make(faults={repr(domain.stack("B", "C")): "drop"})
        job.run()

        self.assertEqual(arm.performed, job.trace_actions)
        self.assertEqual(len(job.trace_states), len(job.trace_actions) + 1)

        failures = planner.Plan(job.trace_states, job.trace_actions,
                                job.trace_ok).failures()
        self.assertEqual([str(a) for _, a in failures], ["stack(B,C)"])

    def test_every_step_except_the_flagged_one_is_a_gamma_edge(self):
        job, world, arm = make(faults={repr(domain.stack("B", "C")): "drop"})
        job.run()
        for i, action in enumerate(job.trace_actions):
            before, after = job.trace_states[i], job.trace_states[i + 1]
            if job.trace_ok[i]:
                self.assertEqual(domain.apply_action(before, action), after)
            else:
                self.assertNotEqual(domain.apply_action(before, action), after)

    def test_the_report_marks_the_failure(self):
        job, world, arm = make(faults={repr(domain.stack("B", "C")): "drop"})
        job.run()
        text = planner.Plan(job.trace_states, job.trace_actions,
                            job.trace_ok).format_run()
        self.assertIn("did not have its modelled effect", text)

    def test_a_clean_run_is_a_valid_path_through_E(self):
        """With no faults there is nothing to flag, and r is a real run."""
        job, world, arm = make()
        job.run()
        self.assertTrue(all(job.trace_ok))
        state = job.trace_states[0]
        for action in job.trace_actions:
            state = domain.apply_action(state, action)
        self.assertEqual(state, job.trace_states[-1])

    def test_recovery_replans_from_where_the_block_landed(self):
        """The re-grasp must aim at the new position, not the original one."""
        job, world, arm = make(faults={repr(domain.stack("B", "C")): "drop"})
        job.run()
        self.assertEqual(world.table_xy["C"],
                         scenarios.SCENARIOS["model"].table_xy["C"])
        # B was dropped at (0.42, -0.28); the second pickup had to use that.
        self.assertEqual(arm.performed[2], domain.pickup("B", domain.TABLE))


class TestGivingUp(unittest.TestCase):
    def test_an_action_that_never_works_stops_the_run(self):
        faults = {repr(domain.pickup("B", domain.TABLE)): "nothing"}
        # The fault dict is consumed on use, so re-arm it every attempt.
        job, world, arm = make()
        arm.faults = _Always(repr(domain.pickup("B", domain.TABLE)), "nothing")
        with self.assertRaises(RuntimeError) as caught:
            job.run()
        self.assertIn("failed", str(caught.exception))
        self.assertIn("times in a row", str(caught.exception))

    def test_it_stops_after_max_retries_not_sooner_or_later(self):
        job, world, arm = make()
        arm.faults = _Always(repr(domain.pickup("B", domain.TABLE)), "nothing")
        with self.assertRaises(RuntimeError):
            job.run()
        attempts = [a for a in arm.performed
                    if a == domain.pickup("B", domain.TABLE)]
        self.assertEqual(len(attempts), task.MAX_RETRIES)


class _Always(dict):
    """A fault that never gets consumed, however often it is popped."""

    def __init__(self, key, value):
        super(_Always, self).__init__()
        self.key = key
        self.value = value

    def pop(self, key, default=None):
        return self.value if key == self.key else default


if __name__ == "__main__":
    unittest.main()
