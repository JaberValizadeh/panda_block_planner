"""Turning an action from A into arm motion.

Two operators, so two methods. The motion itself is the approach that
panda_block_stack arrived at - hover with a clearance that backs off until it
plans, straight-line descent with collisions off, grasp, attach to the hand,
straight-line lift - with three changes the planner makes necessary:

  * `stack(x, D)` exists now. The original could only ever place onto another
    block, because a fixed two-transfer sequence never needs the table as a
    buffer. Anything that has to unstack does.
  * the reach ceiling is a function of x rather than one constant, since the
    planner is free to work anywhere on the table and not just over C.
  * targets come from perception each time, not from a tracked dictionary,
    so a block that ended up somewhere unintended is still picked up.
"""

import copy
import sys

import actionlib
import moveit_commander
import rospy
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

from . import domain, geometry

GRIPPER_OPEN = 0.075          # finger separation when open [m]

# The commanded grasp width does NOT set how hard the fingers squeeze, which is
# worth stating because it looks as though it should. In franka_gazebo's
# FrankaGripperSim::grasp the closing target is hardcoded to 0, not to this
# value: the fingers drive shut at force_desired until they stall, the stall is
# what transitions GRASPING -> HOLDING, and `width` is used only to pick the
# closing direction and to decide afterwards whether the action reports success.
# So asking for a width narrower than the block to "preload" the grip changes
# nothing physical - tried, and it is a no-op.
GRIPPER_WIDTH = geometry.BLOCK
GRASP_FORCE = 25.0            # [N]
RELEASE_HEIGHT = 0.003        # let go this high, so the block drops the last bit

# How close to the goal still counts as arrived when the controller reports a
# tolerance violation. Well under half a block, so a real miss still fails.
REACHED_TOLERANCE = 0.01      # [m]

# panda_link8's +Z runs along the flange axis, so pointing the hand at the
# table is a pi rotation about X; the -pi/4 about Z lines the fingers up with
# world Y, matching how the hand is mounted 45 degrees off link8.
DOWNWARD_RPY = (3.14159265, 0.0, -0.78539816)


class Executor(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")

        self.arm.set_end_effector_link("panda_hand_tcp")
        self.arm.set_planning_time(10.0)
        self.arm.set_num_planning_attempts(10)
        # The datasheet limits are aggressive for a stacking demo; slowing
        # down keeps the block from being shaken loose in Gazebo.
        self.arm.set_max_velocity_scaling_factor(0.25)
        self.arm.set_max_acceleration_scaling_factor(0.25)
        self.arm.set_goal_position_tolerance(0.005)
        self.arm.set_goal_orientation_tolerance(0.02)

        # franka_gazebo exposes the hand as two actions, not a MoveIt group.
        self.gripper_move = actionlib.SimpleActionClient(
            "franka_gripper/move", MoveAction)
        self.gripper_grasp = actionlib.SimpleActionClient(
            "franka_gripper/grasp", GraspAction)
        rospy.loginfo("Waiting for franka_gripper action servers...")
        self.gripper_move.wait_for_server()
        self.gripper_grasp.wait_for_server()

        self.planning_frame = self.arm.get_planning_frame()
        self.held = None
        rospy.loginfo("Planning frame: %s", self.planning_frame)

    # -- planning scene ----------------------------------------------------
    def _box_pose(self, x, y, z):
        p = PoseStamped()
        p.header.frame_id = self.planning_frame
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation.w = 1.0
        return p

    def add_table(self):
        self.scene.add_box("table", self._box_pose(*geometry.TABLE_CENTER),
                           size=geometry.TABLE_SIZE)

    def sync_scene(self, observation):
        """Re-publish every block into MoveIt at its *observed* pose.

        Done before each action rather than once at startup: if a block moved
        for a reason nobody planned, the planner should route around where it
        actually is.
        """
        for block, pose in observation.poses.items():
            if block == self.held:
                continue
            self.scene.add_box("block_%s" % block, self._box_pose(*pose),
                               size=(geometry.BLOCK,) * 3)
        rospy.sleep(0.5)

    def _block_name(self, block):
        return "block_%s" % block

    # -- gripper -----------------------------------------------------------
    def open_gripper(self):
        self.gripper_move.send_goal_and_wait(
            MoveGoal(width=GRIPPER_OPEN, speed=0.1), rospy.Duration(5.0))
        rospy.loginfo("Gripper opened")

    def close_gripper(self):
        goal = GraspGoal()
        goal.width = GRIPPER_WIDTH
        goal.speed = 0.05
        goal.force = GRASP_FORCE
        # Generous: the block is squeezed slightly, and the action reports
        # failure if the final width lands outside this band.
        goal.epsilon.inner = 0.02
        goal.epsilon.outer = 0.02
        self.gripper_grasp.send_goal_and_wait(goal, rospy.Duration(8.0))
        rospy.loginfo("Gripper closed")
        # Give gazebo_grasp_fix its few update cycles to latch the block.
        rospy.sleep(1.5)

    # -- motion ------------------------------------------------------------
    def position_error(self, x, y, z):
        p = self.arm.get_current_pose().pose.position
        return ((p.x - x) ** 2 + (p.y - y) ** 2 + (p.z - z) ** 2) ** 0.5

    def move_to(self, x, y, z, label, required=True):
        q = quaternion_from_euler(*DOWNWARD_RPY)
        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.arm.set_pose_target(pose)
        rospy.loginfo("Moving to %s (%.3f, %.3f, %.3f)", label, x, y, z)
        ok = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        if not ok:
            # go() also returns False when the trajectory finished but the
            # controller declared GOAL_TOLERANCE_VIOLATED - routinely by a
            # fraction of a millimetre under franka_gazebo. Ask where the arm
            # actually ended up before calling it a failure.
            error = self.position_error(x, y, z)
            if error <= REACHED_TOLERANCE:
                rospy.logwarn("%s: planner reported failure but the arm is "
                              "%.1f mm from the goal - continuing",
                              label, error * 1000.0)
                return True
            if not required:
                rospy.logwarn("%s: unreachable (off by %.0f mm)",
                              label, error * 1000.0)
                return False
            raise RuntimeError("Failed to reach %s (off by %.0f mm)"
                               % (label, error * 1000.0))
        return True

    def move_above(self, x, y, z, label):
        """Hover over (x, y, z), backing the clearance off until one plans.

        Returns the clearance used, so the caller descends by exactly that.
        Clearances that would breach the reach envelope at this x are skipped
        rather than attempted: a doomed attempt costs ~10 s and was twice seen
        to jostle the carried block loose.
        """
        ceiling = geometry.max_tcp_z(x, y)
        usable = [c for c in geometry.APPROACH_FALLBACKS if z + c <= ceiling]
        if not usable:
            rospy.logwarn("%s: even %.0f mm of clearance is above the reach "
                          "ceiling (%.3f at x = %.2f, y = %+.2f) - trying "
                          "anyway", label, geometry.MIN_CLEARANCE * 1000.0,
                          ceiling, x, y)
            usable = [geometry.MIN_CLEARANCE]
        for clearance in usable:
            if self.move_to(x, y, z + clearance,
                            "%s (+%.0f mm)" % (label, clearance * 1000.0),
                            required=clearance == usable[-1]):
                return clearance
        raise RuntimeError("Failed to hover above %s" % label)  # not reached

    def move_straight(self, dx, dy, dz, label, avoid_collisions=True):
        """Cartesian move, so the hand never swings sideways near a block.

        Approach and place segments deliberately drive the payload into
        contact, so they pass avoid_collisions=False; the planner refuses the
        last few centimetres otherwise.

        MoveIt 1.1's compute_cartesian_path is (waypoints, eef_step,
        avoid_collisions) - the old jump_threshold argument is gone.
        """
        target = copy.deepcopy(self.arm.get_current_pose().pose)
        target.position.x += dx
        target.position.y += dy
        target.position.z += dz

        plan, fraction = self.arm.compute_cartesian_path(
            [target], 0.005, avoid_collisions)
        if fraction < 0.9:
            rospy.logwarn("%s: only %.0f%% of the straight path was planned, "
                          "falling back to a joint-space move",
                          label, fraction * 100)
            return self.move_to(target.position.x, target.position.y,
                                target.position.z, label)
        rospy.loginfo("Cartesian move: %s", label)
        ok = self.arm.execute(plan, wait=True)
        self.arm.stop()
        if ok:
            return True

        # execute() reports ABORTED the same way go() does, and it must be
        # checked the same way move_to already checks go() - a Cartesian plan
        # that compute_cartesian_path followed 100% can still be halted by the
        # controller with GOAL_TOLERANCE_VIOLATED, and the return value is the
        # only place it surfaces (move_group logs "Execution completed:
        # ABORTED" to the launch console, not to this node).
        #
        # The re-plan loop in task.py is a genuine safety net for a placement
        # that lands wrong, but it does NOT cover a silently-aborted *descent
        # onto a block*: the executor would go on to close the gripper on air,
        # set self.held, and attach a phantom box - and because perceive() is
        # told held=self.held, the block still sitting on the table is then
        # abstracted as holding(block), which matches the expected post-pickup
        # state exactly. The loop would record a grasp that never happened and
        # move on. So the check has to be here, where the intended target is
        # still known, not left to a verification step the masking defeats.
        error = self.position_error(target.position.x, target.position.y,
                                    target.position.z)
        if error <= REACHED_TOLERANCE:
            rospy.logwarn("%s: execution aborted but the arm is %.1f mm from "
                          "the goal - continuing", label, error * 1000.0)
            return True
        rospy.logwarn("%s: execution aborted %.0f mm from the goal, replanning "
                      "in joint space", label, error * 1000.0)
        return self.move_to(target.position.x, target.position.y,
                            target.position.z, label)

    def go_home(self):
        self.arm.set_named_target("ready")
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    # -- the two operators -------------------------------------------------
    def pickup(self, block, pose):
        """pickup(x): grasp `block` where it was observed, and lift it clear."""
        x, y, z = pose
        rospy.loginfo("--- pickup(%s) at (%.3f, %.3f, %.3f)", block, x, y, z)

        clearance = self.move_above(x, y, z, "above %s" % block)
        # The gripper is about to occupy the block's space on purpose.
        self.scene.remove_world_object(self._block_name(block))
        rospy.sleep(0.5)

        self.move_straight(0, 0, -clearance, "descend onto %s" % block,
                           avoid_collisions=False)
        self.close_gripper()

        # Tell MoveIt the block travels with the hand so it is collision
        # checked on the way to wherever it is going.
        self.scene.add_box(self._block_name(block), self._box_pose(x, y, z),
                           size=(geometry.BLOCK,) * 3)
        rospy.sleep(0.5)
        self.arm.attach_object(
            self._block_name(block), "panda_hand",
            touch_links=self.robot.get_link_names(group="panda_hand"))
        rospy.sleep(0.5)
        self.held = block

        self.move_straight(0, 0, clearance, "lift %s" % block)

    def place(self, block, x, y, z, label):
        """Put the carried block down with its centre at (x, y, z)."""
        rospy.loginfo("--- place %s at (%.3f, %.3f, %.3f) [%s]",
                      block, x, y, z, label)
        clearance = self.move_above(x, y, z, label)
        self.move_straight(0, 0, -(clearance - RELEASE_HEIGHT),
                           "descend to %s" % label, avoid_collisions=False)

        self.open_gripper()
        self.arm.detach_object(self._block_name(block))
        self.held = None
        rospy.sleep(0.5)
        self.scene.remove_world_object(self._block_name(block))

        self.move_straight(0, 0, clearance, "retreat from %s" % label)

    def execute(self, action, observation, slot_hint=None):
        """Run one action from A against the world as just observed."""
        if action.kind == "pickup":
            self.pickup(action.block, observation.poses[action.block])
            return

        if action.support == domain.TABLE:
            x, y = slot_hint
            z = geometry.BLOCK_Z
            label = "table slot (%.2f, %.2f)" % (x, y)
        else:
            x, y, sz = observation.poses[action.support]
            z = geometry.place_z_on(sz)
            label = "top of %s" % action.support
        self.place(action.block, x, y, z, label)

    def target_of(self, action, observation, slot_hint=None):
        """Where `action` intends to leave its block - for verifying afterwards."""
        if action.kind == "pickup":
            return None
        if action.support == domain.TABLE:
            return (slot_hint[0], slot_hint[1], geometry.BLOCK_Z)
        x, y, sz = observation.poses[action.support]
        return (x, y, geometry.place_z_on(sz))
