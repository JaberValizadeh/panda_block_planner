#!/usr/bin/env python3
"""Teleport the blocks into a scenario's starting layout.

    rosrun panda_block_planner setup_scenario.py _scenario:=sussman

Run this between tasks instead of restarting Gazebo. The layout comes from
scenarios.py, the same place the planner's initial state comes from, so the
two cannot drift apart - which matters, because a scenario whose symbolic
state does not match the blocks on the table produces a plan that fails on its
first action for reasons that look like a motion bug.
"""

import sys

import rospy

from panda_block_planner import scene_setup, scenarios


def main():
    rospy.init_node("setup_scenario", anonymous=True)
    name = rospy.get_param("~scenario", "model")
    if name not in scenarios.SCENARIOS:
        rospy.logerr("unknown scenario '%s'; known: %s", name,
                     ", ".join(sorted(scenarios.SCENARIOS)))
        sys.exit(1)
    if name not in scenarios.PHYSICAL:
        rospy.logerr("'%s' needs blocks the world does not have (it has %s). "
                     "Use plan_only.py for it.", name,
                     ", ".join(scenarios.WORLD_BLOCKS))
        sys.exit(1)

    scenario = scenarios.SCENARIOS[name]
    rospy.loginfo("Setting up '%s': %s", name, scenario.description)
    failed = scene_setup.place_scenario(scenario)
    if failed:
        rospy.logerr("Could not place: %s", ", ".join(failed))
        sys.exit(1)
    rospy.loginfo("Scenario '%s' is set up. Goal: %s", name, scenario.goal_text)
    if scenario.stowed:
        rospy.loginfo("Stowed at the back of the table, not in play: %s",
                      ", ".join(scenario.stowed))


if __name__ == "__main__":
    main()
