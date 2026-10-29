#!/usr/bin/env python3

import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, MoveItErrorCodes


# --- Arm poses (joint angles in radians) ---
arm_pick = [1.57, -0.79, -1.57, 0.79, 0.0, 0.0]
arm_stow = [0.0, -0.26, -2.36, 1.05, 0.0, 0.0]   # stowed pose: use while driving
arm_place = [3.1416, -0.0, -1.57, 0.0, 0.0, 0.0]

nav_conveyor_c = [5.4, 0.7, -1.5708]
nav_conveyor_b = [5.4, 0.2, -1.5708]
nav_conveyor_a = [5.4, -1.4, -1.5708]
nav_conveyor_turn = [5.4, -1.4, 0.0]
nav_approach_place = [30.0, -2.0, 0.0]
nav_turn_place = [30.0, -2.0, -1.5708]
nav_place = [32.0, -5.45, -1.5708]
nav_exit_turn = [32.0, -5.45, 1.5708]
nav_exit = [32.0, -2.0, 1.5708]



class HSCombinedRunner(Node):
    JOINT_NAMES: List[str] = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    PLANNING_GROUP: str = 'tmr_arm'
    ARM_ACTION: str = '/move_action'
    NAV_ACTION: str = 'navigate_to_pose'

    def __init__(self):
        super().__init__('hs_combined_runner')

        self.nav_client = ActionClient(self, NavigateToPose, self.NAV_ACTION)
        self.arm_client = ActionClient(self, MoveGroup, self.ARM_ACTION)

        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.arm_client.wait_for_server()
        self.get_logger().info('Both action servers ready.')

    # ------------------------------------------------------------------ NAV --
    @staticmethod
    def nav_pose(x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)
        return ps

    def navigate(self, x: float, y: float, yaw: float) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = self.nav_pose(x, y, yaw)
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        def feedback_cb(fb):
            self.get_logger().info(f'Distance remaining: {fb.feedback.distance_remaining:.2f} m',
                                   throttle_duration_sec=2.0)

        send_fut = self.nav_client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Nav goal rejected.')
            return False

        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result()
        if res is None or res.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('Navigation failed.')
            return False

        self.get_logger().info(f'Reached ({x}, {y}, yaw {yaw:.2f}).')
        return True

    # ------------------------------------------------------------------ ARM --
    def _arm_goal(self, joints: List[float]) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        cs = Constraints()
        for name, val in zip(self.JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            cs.joint_constraints.append(jc)

        req.goal_constraints.append(cs)
        goal.request = req
        goal.planning_options.plan_only = False  # plan + execute
        return goal

    def move_arm(self, joints: List[float]) -> bool:
        send_fut = self.arm_client.send_goal_async(self._arm_goal(joints))
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Arm goal rejected.')
            return False

        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result()
        ok = (res is not None
              and res.status == GoalStatus.STATUS_SUCCEEDED
              and res.result.error_code.val == MoveItErrorCodes.SUCCESS)
        if not ok:
            self.get_logger().error('Arm motion failed.')
        return ok

    # ------------------------------------------------------------- SEQUENCE --
    def run(self):
        # Stow the arm before driving anywhere
        if not self.move_arm(arm_stow): return

        # Move to start of conveyor
        self.navigate(nav_conveyor_c)
         
        # Move arm to pick position
        if not self.move_arm(arm_pick): return
        
        # Move to end of conveyor past both other box locations  
        if not self.navigate(nav_conveyor_b): return
        if not self.navigate(nav_conveyor_a): return
         
        # Stow the arm before driving
        if not self.move_arm(arm_stow): return

        # Move to shelf
        if not self.navigate(nav_conveyor_turn): return
        if not self.navigate(nav_approach_place): return
        if not self.navigate(nav_turn_place): return
        if not self.navigate(nav_place): return

        # Place the box and stow the arm before driving
        if not self.move_arm(arm_place): return
        if not self.move_arm(arm_stow): return

        # Navigate out of shelf area
        if not self.navigate(nav_exit_turn): return
        if not self.navigate(nav_exit): return

        time.sleep(0.0)

        self.get_logger().info('Sequence complete ✅')


def main(args=None):
    rclpy.init(args=args)
    node = HSCombinedRunner()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
