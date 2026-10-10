"""Put the blocks where a scenario says they start.

Lives here rather than in the script so the task node can call it too. Under
roslaunch there is no ordering between nodes, so "set the scene up, then run
the task" cannot be expressed as two <node> tags - the task would race the
setup and plan against whatever the blocks happened to be doing. Calling this
from inside the task node makes the ordering a fact rather than a hope.

A teleport is not a placement. Dropping a block onto another one with
set_model_state puts their faces exactly in contact, and ODE resolves that
first contact by pushing - usually by nothing, occasionally by enough to slide
the upper block off. Measured on the five-block layout: the first pass after a
fresh world load left E 54-71 mm off C and C 17 mm off its own target, while
every later pass, and every retry, was exact to 0.1 mm. So each pass is checked
against ground truth and repeated if a block did not stay where it was put.

Without that the run starts from a state nobody asked for - survivable, since
the loop re-plans from what it perceives, but it means the scenario on the
table is not the scenario that was named. That is exactly what happened the
first time `five_blocks` was run: E was on the table rather than on C, and the
robot solved the ten-action problem it could see instead of the twelve-action
one it was given.
"""

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from . import geometry, world_model

SETTLE = 0.3      # let each block come to rest before the next lands on it
ATTEMPTS = 3      # passes before giving up on a block that will not stay put

# A teleport lands exactly, so anything that moved at all was pushed by
# something. A quarter of a block is well inside the half-block that would
# start reading as a different position, and well outside solver noise.
TOLERANCE = geometry.BLOCK / 4.0


def place_scenario(scenario, timeout=15.0, attempts=ATTEMPTS):
    """Teleport every block in the world to its pose for `scenario`.

    Every block, not only the ones the scenario is about: `block_poses` puts
    the unused ones on the stow strip at the back of the table. Placing only
    the named blocks would leave the rest wherever the previous run dropped
    them, which for a block abandoned mid-table is something the arm will
    later drive through.

    Returns the list of blocks that could not be placed, empty on success.
    """
    rospy.wait_for_service("/gazebo/set_model_state", timeout=timeout)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    # No `ignore`: this is about where the cubes physically are, and the stowed
    # ones are placed here too, so they are checked here too.
    world = world_model.WorldModel()

    targets = scenario.block_poses()
    for attempt in range(1, attempts + 1):
        refused = _teleport(set_state, targets)
        if refused:
            return refused
        world.settle()
        moved = _did_not_stay(world, targets)
        if not moved:
            return []
        rospy.logwarn("pass %d: %s did not stay where it was put; placing the "
                      "layout again", attempt,
                      ", ".join("%s (%.0f mm out)" % (b, d * 1000.0)
                                for b, d in moved))
    return ["%s (will not stay put)" % b for b, _ in moved]


def _teleport(set_state, targets):
    """One pass over the layout. Returns the blocks Gazebo refused to move."""
    refused = []
    # Bottom-up, so a block never lands on a support that is not there yet.
    for block, (x, y, z) in sorted(targets.items(), key=lambda kv: kv[1][2]):
        state = ModelState()
        state.model_name = "block_%s" % block
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.w = 1.0
        # The twist is left at zero: a block that was still moving would
        # otherwise keep its velocity and drift straight back off the table.
        resp = set_state(state)
        if resp.success:
            rospy.loginfo("  block_%s -> (%.3f, %.3f, %.3f)", block, x, y, z)
        else:
            refused.append("%s (%s)" % (block, resp.status_message))
        rospy.sleep(SETTLE)
    return refused


def _did_not_stay(world, targets):
    """[(block, distance), ...] for blocks that are not where they were put."""
    poses = world.poses()
    out = []
    for block, target in sorted(targets.items()):
        if block not in poses:
            out.append((block, float("inf")))
            continue
        error = geometry.distance(poses[block], target)
        if error > TOLERANCE:
            out.append((block, error))
    return out
