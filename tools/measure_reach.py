#!/usr/bin/env python3
"""Measure the reach envelope instead of extrapolating it.

    rosrun panda_block_planner measure_reach.py          # needs move_group up

`geometry.max_tcp_z` is one measured point and a straight line drawn through
it on an assumption about how the Panda's reach falls off. This asks the robot
instead: for each (x, y) on the table, what is the highest z at which a
top-down TCP pose still has an IK solution?

IK, not planning. A planning attempt is the honest test and is what produced
the original number, but it costs ~10 s per *failure* and a grid this size
would take hours. A pose with no IK solution is unplannable for certain, and
the boundary IK reports is then spot-checked with real planning calls by
--verify. The orientation is DOWNWARD_RPY, the only one the executor ever
asks for, because reach depends on it.

Writes a Python literal to stdout, for pasting into geometry.py.
"""

import argparse
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from tf.transformations import quaternion_from_euler

DOWNWARD_RPY = (3.14159265, 0.0, -0.78539816)
GROUP = "panda_arm"
LINK = "panda_hand_tcp"


def probe(ik, x, y, z, frame, timeout=0.05):
    request = PositionIKRequest()
    request.group_name = GROUP
    request.ik_link_name = LINK
    request.avoid_collisions = False
    request.timeout = rospy.Duration(timeout)

    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    q = quaternion_from_euler(*DOWNWARD_RPY)
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = q
    request.pose_stamped = pose

    try:
        return ik(request).error_code.val == MoveItErrorCodes.SUCCESS
    except rospy.ServiceException as exc:
        rospy.logwarn("IK service failed at (%.2f, %.2f, %.3f): %s", x, y, z, exc)
        return False


def ceiling(ik, x, y, frame, lo, hi, step, attempts):
    """Highest z in [lo, hi] that solves. None if even `lo` does not."""
    best = None
    z = lo
    while z <= hi + 1e-9:
        # IK is randomly seeded, so one failure is not proof of unreachable.
        if any(probe(ik, x, y, z, frame) for _ in range(attempts)):
            best = z
        z += step
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, nargs="+",
                        default=[0.36, 0.42, 0.48, 0.54, 0.60, 0.66, 0.72])
    parser.add_argument("--y", type=float, nargs="+",
                        default=[-0.28, -0.20, -0.12, 0.0, 0.12, 0.20, 0.28])
    parser.add_argument("--lo", type=float, default=0.44)
    parser.add_argument("--hi", type=float, default=0.92)
    parser.add_argument("--step", type=float, default=0.01)
    parser.add_argument("--attempts", type=int, default=4,
                        help="IK seeds per pose before calling it unreachable")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("measure_reach", anonymous=True)
    rospy.loginfo("waiting for /compute_ik ...")
    rospy.wait_for_service("/compute_ik", timeout=60.0)
    ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)
    frame = rospy.get_param("/move_group/planning_frame", "world")

    table = {}
    for x in args.x:
        row = []
        for y in args.y:
            z = ceiling(ik, x, y, frame, args.lo, args.hi, args.step,
                        args.attempts)
            row.append(z)
            rospy.loginfo("x=%.2f y=%+.2f -> %s", x, y,
                          "%.3f" % z if z is not None else "unreachable")
            sys.stdout.flush()
        table[x] = row

    print()
    print("REACH_Y = %r" % (tuple(args.y),))
    print("REACH_X = %r" % (tuple(args.x),))
    print("REACH_Z = (")
    for x in args.x:
        cells = ", ".join("None" if z is None else "%.3f" % z
                          for z in table[x])
        if len(args.y) == 1:
            cells += ","          # a one-column sweep is still a tuple
        print("    (%s),   # x = %.2f" % (cells, x))
    print(")")


if __name__ == "__main__":
    main()
