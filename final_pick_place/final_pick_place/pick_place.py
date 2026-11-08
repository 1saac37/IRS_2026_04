#!/usr/bin/env python3

import math
import time
import json
import re
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose, BackUp
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, MoveItErrorCodes


# --- Arm poses (joint angles in radians) ---
arm_pick = [1.57, -0.79, -1.57, 0.79, 0.0, 0.0]
arm_stow = [0.0, -0.26, -2.36, 1.05, 0.0, 0.0]   # stowed pose: use while driving
arm_place = [3.1416, -0.0, -1.57, 0.0, 0.0, 0.0]

nav_conveyor_c = [5.4, 0.7, -1.5708]
nav_conveyor_b = [5.4, 0.2, -1.5708]
nav_conveyor_a = [5.4, -1.4, -1.5708]

nav_conveyor_turn = [5.4, -1.4, 0.0] # not neccessary after reverse implemented

nav_approach_place = [30.7, -2.8, -1.5708]
nav_turn_place = [30.7, -2.0, -1.5708]
nav_place = [30.7, -5.9, -1.5708]
nav_exit_turn = [30.7, -1.0, 3.1416]
nav_exit = [16.0, 0.0, 3.1416]

# Pickup leg distance
PICKUP_LEG_DISTANCE = abs(nav_conveyor_c[1] - nav_conveyor_a[1])  # 2.1 m

# Distance to reverse after successful pickup to allow clearance to head towards shelf
REVERSE_AFTER_PICK = (0.5 * PICKUP_LEG_DISTANCE) 

# Distance to reverse after reaching nav_place
REVERSE_AFTER_PLACE_DISTANCE = 4.0

class HSCombinedRunner(Node):
    JOINT_NAMES: List[str] = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    PLANNING_GROUP: str = 'tmr_arm'
    ARM_ACTION: str = '/move_action'
    NAV_ACTION: str = 'navigate_to_pose'
    BACKUP_ACTION: str = 'backup'

    def __init__(self):
        super().__init__('hs_combined_runner')

        self.nav_client = ActionClient(self, NavigateToPose, self.NAV_ACTION)
        self.arm_client = ActionClient(self, MoveGroup, self.ARM_ACTION)
        self.backup_client = ActionClient(self, BackUp, self.BACKUP_ACTION)

        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.arm_client.wait_for_server()
        self.get_logger().info('Waiting for BackUp action server...')
        self.backup_client.wait_for_server()
        self.get_logger().info('All action servers ready.')

        # --- Weight tracking from HMI status ---
        self._latest_weight = None  # None = N/A / no reading yet
        self.create_subscription(String, '/hmi/unified_status', self._hmi_status_cb, 10)

    
    @staticmethod
    def nav_pose(x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = x
        ps.pose.position.y = y

        half = yaw * 0.5
        ps.pose.orientation.z = math.sin(half)
        ps.pose.orientation.w = math.cos(half)
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

    
    def reverse(self, distance: float, speed: float = 0.7) -> bool:
        """Drive straight backward `distance` meters at the current heading — no rotation."""
        self.get_logger().info(f'Backing up {distance:.2f} m...')
        self.backup_client.wait_for_server()

        goal = BackUp.Goal()
        goal.target.x = -abs(distance)      # negative x = backward in robot frame
        goal.speed = -abs(speed)            # negative speed = reverse
        goal.time_allowance = Duration(sec=10)

        send_fut = self.backup_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Backup goal was rejected!')
            return False

        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return res_fut.result() is not None

    
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

    
    @staticmethod
    def _parse_weight(status_json: str):
        """Return weight in kg as a float, or None if N/A / unparsable."""
        try:
            data = json.loads(status_json)
            weight_raw = data.get('box', {}).get('weight_raw', '')
        except (json.JSONDecodeError, AttributeError):
            return None
        match = re.search(r'(-?\d+(?:\.\d+)?)\s*kg', weight_raw)
        return float(match.group(1)) if match else None

    def _hmi_status_cb(self, msg: String):
        self._latest_weight = self._parse_weight(msg.data)

    def weight_is_positive(self) -> bool:
        return self._latest_weight is not None and self._latest_weight > 0.0

    def wait_for_weight_positive(self):
        self.get_logger().info('Waiting for a box to be weighed...')
        while rclpy.ok() and not self.weight_is_positive():
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f'Box detected, weight={self._latest_weight} kg')

    
    def pickup_leg(self) -> bool:
        """Stow -> conveyor start -> pick -> move past b and a. Returns False on any step failure."""
        if not self.move_arm(arm_stow): return False
        if not self.navigate(*nav_conveyor_c): return False
        if not self.move_arm(arm_pick): return False
        if not self.navigate(*nav_conveyor_b): return False
        if not self.navigate(*nav_conveyor_a): return False
        return True

    def delivery_leg(self) -> bool:
        """Stow -> shelf approach -> place -> stow -> exit. Returns False on any step failure."""
        if not self.move_arm(arm_stow): return False
        if not self.reverse(distance=REVERSE_AFTER_PICK): return False 
        # if not self.navigate(*nav_conveyor_turn): return False  # not neccessary anymore
        if not self.navigate(*nav_approach_place): return False
        # if not self.navigate(*nav_turn_place): return False # not neccessary
        if not self.move_arm(arm_place): return False
        if not self.navigate(*nav_place): return False
        if not self.reverse(distance=REVERSE_AFTER_PLACE_DISTANCE): return False  
        if not self.move_arm(arm_stow): return False
        # if not self.navigate(*nav_exit_turn): return False
        if not self.navigate(*nav_exit): return False
        return True

    def return_to_pickup_start(self) -> bool:
        """Undo the pickup leg without rotating: reverse straight back to nav_conveyor_c."""
        return self.reverse(distance=PICKUP_LEG_DISTANCE)

    def run(self):
        try:
            while rclpy.ok():
                self.wait_for_weight_positive()

                while True:
                    if not self.pickup_leg():
                        self.get_logger().error('Pickup leg failed on a nav/arm step. Aborting run.')
                        return

                    rclpy.spin_once(self, timeout_sec=0.2)  # fresh weight reading
                    if not self.weight_is_positive():
                        break
                    self.get_logger().warn('Still weighted after pickup leg - retrying.')
                    if not self.return_to_pickup_start():
                        self.get_logger().error('Reverse maneuver failed. Aborting run.')
                        return

                if not self.delivery_leg():
                    self.get_logger().error('Delivery leg failed on a nav/arm step. Aborting run.')
                    return

                self.get_logger().info('Delivery cycle complete. Waiting for next box...')
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user.')


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