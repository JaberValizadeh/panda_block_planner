#!/usr/bin/env python3
"""Perceive, plan, act, verify - one action at a time.

    rosrun panda_block_planner plan_and_stack.py _scenario:=sussman

The loop itself lives in panda_block_planner.task, so it can be tested against
fakes without a simulator. This script is only the wiring: pick a scenario,
build the real world model and the real arm, and hand them over.
"""

import sys

import moveit_commander
import rospy

from panda_block_planner import executor, scenarios, scene_setup, task
from panda_block_planner import world_model


def main():
    rospy.init_node("plan_and_stack", anonymous=True)
    name = rospy.get_param("~scenario", "model")
    if name not in scenarios.SCENARIOS:
        rospy.logerr("unknown scenario '%s'; known: %s", name,
                     ", ".join(sorted(scenarios.SCENARIOS)))
        sys.exit(1)
    scenario = scenarios.SCENARIOS[name]

    # Setting the scene up from inside the task node, rather than as a second
    # <node> tag, is what makes "blocks placed, then planned against" ordered
    # instead of a race.
    if rospy.get_param("~setup", False):
        if name not in scenarios.PHYSICAL:
            rospy.logerr("'%s' names blocks the world does not have (it has "
                         "%s)", name, ", ".join(scenarios.WORLD_BLOCKS))
            sys.exit(1)
        rospy.loginfo("Placing blocks for '%s'", name)
        failed = scene_setup.place_scenario(scenario)
        if failed:
            rospy.logerr("Could not place: %s", ", ".join(failed))
            sys.exit(1)

    # The world spawns all five blocks whatever the scenario needs; the ones
    # it does not name are stowed out of the way and filtered out of
    # perception here, so a three-block problem stays a three-block problem.
    if scenario.stowed:
        rospy.loginfo("Not in play for '%s': %s", name,
                      ", ".join(scenario.stowed))
    world = world_model.WorldModel(ignore=scenario.stowed)

    job = task.Task(scenario, world, executor.Executor())
    try:
        job.run()
    except Exception as exc:
        rospy.logerr("Task failed: %s", exc)
        raise
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
