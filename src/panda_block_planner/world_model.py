"""Gazebo ground truth -> a state in E, plus the metric poses behind it.

Two things come back from every observation and both matter:

  * the symbolic state, which is what the planner reasons over;
  * the actual block poses, which is what the arm has to be driven to.

Keeping only the first is the mistake that makes a demo report success at an
empty table. A block that slips out of the gripper and lands flat on the table
is symbolically indistinguishable from one that was never picked up - the
abstraction has no way to say "on the table, but 200 mm from where you think".
The metric half is what notices.

Gripper state is proprioception, not perception: Gazebo can tell us where the
blocks are, but which one is between the fingers is something the executor
knows and tells us. Hence the `held` argument.

`ignore` is the other thing the world cannot tell us. Gazebo spawns all five
blocks whatever the scenario is about, so the ones a three-block scenario is
not about - stowed on the far strip of the table, see `geometry.stow_xy` - are
named here and dropped before abstraction. Left in, they would join the state
and change the problem: a spare cube gives C somewhere to go that is not the
table, and the Sussman anomaly stops anomalising.
"""

import rospy
from gazebo_msgs.msg import ModelStates

from . import domain, geometry

BLOCK_PREFIX = "block_"

# A block counts as sitting at a level if it is within this of the exact
# height. Half a block would be ambiguous; a third is comfortably decisive.
LEVEL_TOLERANCE = geometry.BLOCK / 3.0
# ... and as resting on the one below if their centres line up this closely.
STACK_XY_TOLERANCE = geometry.BLOCK * 0.6

# A block moving faster than this has not finished landing. Perceiving the
# instant an action returns reads the world mid-flight: a block released onto a
# tower bounces a few mm, and abstracting it mid-bounce reads a height it is
# only passing through.
#
# One quiet sample is not proof of rest, so stillness has to PERSIST for
# SETTLE_DWELL. That requirement was originally sized to outlast something far
# worse - gazebo_grasp_fix holding a block welded to the palm for 1.5 s after
# the fingers opened, during which it hung motionless partway through the
# retreat and would have passed any instantaneous velocity check. That bug is
# fixed at source now (the grasp-fix timing in panda_grasp.urdf.xacro - now
# update_rate 10, grip_count_threshold 1, max_grip_count 3; see design note 4
# of panda_block_stack's README), but a dwell is still the right shape for
# this test, and 0.75 s costs nothing next to the motion it follows.
SETTLE_SPEED = 0.005          # [m/s]
SETTLE_DWELL = 0.75           # [s] stillness must persist this long
SETTLE_TIMEOUT = 8.0          # [s]


class Observation(object):
    def __init__(self, state, poses, problems):
        self.state = state
        self.poses = poses          # {"A": (x, y, z), ...}
        self.problems = problems

    def xy(self, block):
        x, y, _ = self.poses[block]
        return (x, y)

    @property
    def held(self):
        """The block in the hand, or None.

        Read off the state rather than remembered, so anything holding an
        Observation has the answer without also having the arm.
        """
        for atom in self.state:
            if atom[0] == "holding":
                return atom[1]
        return None

    def table_xy(self):
        """Planar positions of the blocks resting directly on the table."""
        return {b: self.xy(b) for b in domain.blocks_in(self.state)
                if domain.support_of(self.state, b) == domain.TABLE}

    def occupied_xy(self):
        """Planar positions of everything actually resting on the table.

        The held block is not one of them. Perception still reports an xy for
        it - wherever the hand has carried it to - but that is where the arm
        is, not a place anything is sitting, and treating it as occupied space
        rules out whatever slot the gripper happens to be over.
        """
        held = self.held
        return [self.xy(b) for b in self.poses if b != held]


class WorldModel(object):
    def __init__(self, timeout=5.0, ignore=()):
        self.timeout = timeout
        self.ignore = frozenset(ignore)

    def _raw_states(self):
        return rospy.wait_for_message("/gazebo/model_states", ModelStates,
                                      timeout=self.timeout)

    def _in_play(self, name):
        return (name.startswith(BLOCK_PREFIX)
                and name[len(BLOCK_PREFIX):] not in self.ignore)

    def poses(self):
        """{"A": (x, y, z), ...} for every block in play, straight from Gazebo.

        Public because setting the scene up needs the poses without the
        abstraction over them: `scene_setup` checks that a teleport landed, and
        "on the table 55 mm from where it was put" is not something a state in
        E can say.
        """
        states = self._raw_states()
        return {name[len(BLOCK_PREFIX):]: (p.position.x, p.position.y,
                                           p.position.z)
                for name, p in zip(states.name, states.pose)
                if self._in_play(name)}

    def settle(self, held=None, timeout=SETTLE_TIMEOUT):
        """Wait until no free block is moving, so perception sees a still world.

        `held` is excluded: a block in the gripper jitters against the fingers
        for as long as it is carried, and waiting for it to stop would only
        ever time out.

        Returns True if the scene came to rest, False if it was still moving
        when `timeout` ran out - the caller perceives either way, because a
        world that will not settle is exactly the kind of problem the metric
        half of an Observation exists to report.
        """
        deadline = rospy.get_time() + timeout
        still_since = None
        while not rospy.is_shutdown():
            states = self._raw_states()
            speeds = [(t.linear.x ** 2 + t.linear.y ** 2 + t.linear.z ** 2) ** 0.5
                      for name, t in zip(states.name, states.twist)
                      if self._in_play(name)
                      and name[len(BLOCK_PREFIX):] != held]
            now = rospy.get_time()
            if not speeds:
                return True
            if max(speeds) < SETTLE_SPEED:
                still_since = now if still_since is None else still_since
                if now - still_since >= SETTLE_DWELL:
                    return True
            else:
                still_since = None
            if now >= deadline:
                rospy.logwarn("blocks still moving after %.1f s (max %.3f m/s)",
                              timeout, max(speeds))
                return False

    def perceive(self, held=None):
        """Read the world and abstract it into a state.

        Raises rospy.ROSException if Gazebo does not answer - never guesses.
        """
        poses = self.poses()
        if not poses:
            raise RuntimeError("no block_* models in /gazebo/model_states "
                               "(ignoring %s)" % (", ".join(sorted(self.ignore))
                                                  or "nothing"))
        return abstract(poses, held=held, source="Gazebo")


def support_under(block, pose, resting):
    """Which of `resting` is directly underneath `block`, if any.

    One level down and centres within STACK_XY_TOLERANCE; nearest wins.
    """
    x, y, z = pose
    best, best_d = None, None
    for other, (ox, oy, oz) in resting.items():
        if other == block:
            continue
        if abs((z - oz) - geometry.BLOCK) > LEVEL_TOLERANCE:
            continue
        d = ((x - ox) ** 2 + (y - oy) ** 2) ** 0.5
        if d <= STACK_XY_TOLERANCE and (best_d is None or d < best_d):
            best, best_d = other, d
    return best


def abstract(poses, held=None, source="the world"):
    """Block poses -> an Observation: a state in E, the poses, and problems.

    This is deliberately independent of where the poses came from. Reading them
    off /gazebo/model_states and inferring them from a camera and a lidar are
    very different problems, but once each block has an (x, y, z) the step up to
    `on(A, B)` is the same arithmetic, and it should not exist twice - a level
    tolerance that drifted between two copies would give the two packages
    different opinions about the same table. panda_sensor_stack builds `poses`
    from sensors and calls straight into here.

    `source` only names the origin of the poses in the problem strings, so that
    "holding A, which the camera cannot see" reads correctly.
    """
    atoms = set()
    problems = []
    if held is None:
        atoms.add(domain.HANDFREE)
    else:
        if held not in poses:
            problems.append("holding %s, which %s does not know" % (held, source))
        atoms.add(domain.holding(held))

    resting = {b: p for b, p in poses.items() if b != held}

    for block, (x, y, z) in sorted(resting.items()):
        if z < geometry.TABLE_TOP - LEVEL_TOLERANCE:
            # Below the table top: it went over the edge.
            problems.append("%s has fallen off the table (z = %.3f)"
                            % (block, z))
            continue

        level = int(round((z - geometry.BLOCK_Z) / geometry.BLOCK))
        if abs(z - geometry.block_z(level)) > LEVEL_TOLERANCE:
            problems.append("%s is at z = %.3f, not on any level" % (block, z))
            continue

        if level <= 0:
            atoms.add(domain.on(block, domain.TABLE))
            continue

        support = support_under(block, (x, y, z), resting)
        if support is None:
            # A level up with nothing underneath: mid-topple, or it is
            # perched on the tower's corner. Either way not a state the
            # planner should be handed.
            problems.append("%s is a level up but rests on nothing" % block)
            continue
        atoms.add(domain.on(block, support))

    state = frozenset(atoms)
    problems.extend(domain.check_state(state))
    return Observation(state, poses, problems)
