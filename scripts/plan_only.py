#!/usr/bin/env python3
"""Plan a scenario and print the run, without ROS, Gazebo or a robot.

    ./plan_only.py                  # every scenario
    ./plan_only.py sussman          # one of them
    ./plan_only.py sussman --naive  # ... ignoring the reach envelope

No rospy import anywhere below this line, which is the point: the reasoning
can be developed and checked in a second, against a package that takes a
minute to bring up and several to run a task through.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "src"))

from panda_block_planner import domain, geometry, planner, scenarios  # noqa: E402


def show(scenario, use_reach_filter=True):
    print("=" * 72)
    print("%s - %s" % (scenario.name, scenario.description))
    print("=" * 72)
    print("  s0   = %s" % domain.format_state(scenario.initial))
    print("  goal = %s" % scenario.goal_text)
    print("  table positions: %s"
          % ", ".join("%s at (%.2f, %.2f)" % (b, x, y)
                      for b, (x, y) in sorted(scenario.table_xy.items())))

    feasible = None
    if use_reach_filter:
        buffer_xy = geometry.free_slot(list(scenario.table_xy.values()))
        feasible = geometry.reach_filter(scenario.table_xy, buffer_xy)
        print("  reach ceiling at y=0.00: %s"
              % ", ".join("%.3f m at x=%.2f" % (geometry.max_tcp_z(x), x)
                          for x in (0.45, 0.55, 0.62, 0.70)))
        print("  ... and at x=0.45:        %s"
              % ", ".join("%.3f m at y=%+.2f" % (geometry.max_tcp_z(0.45, y), y)
                          for y in (0.0, 0.12, 0.20, 0.28)))
    print()

    result = planner.plan(scenario.initial, scenario.goal, feasible=feasible)
    if result is None:
        print("  NO PLAN")
        print("  %s" % planner.explain_failure(scenario.initial,
                                               scenario.goal, feasible))
        print()
        return

    print(indent(result.format_run()))
    print()
    print("  %d actions; |E| reachable from s0 = %d"
          % (len(result), len(domain.reachable_states(scenario.initial))))
    print()


def indent(text):
    return "\n".join("  " + line for line in text.splitlines())


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    use_filter = "--naive" not in sys.argv

    names = args or list(scenarios.SCENARIOS)
    for name in names:
        if name not in scenarios.SCENARIOS:
            print("unknown scenario '%s'; known: %s"
                  % (name, ", ".join(sorted(scenarios.SCENARIOS))))
            return 1
        show(scenarios.SCENARIOS[name], use_filter)
    return 0


if __name__ == "__main__":
    sys.exit(main())
