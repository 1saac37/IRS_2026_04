#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# start changes to copy

from std_msgs.msg import String
import json
import re

# end


# --- Helper function to build a PoseStamped ---
def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    """
    Create a PoseStamped (position + orientation) in the 'map' frame.
    - x, y are coordinates in meters
    - yaw is robot orientation (heading) in radians
    """
    ps = PoseStamped()
    ps.header.frame_id = 'map'  # always use 'map' for navigation goals
    ps.pose.position.x = x
    ps.pose.position.y = y

    # Convert yaw (in radians) into quaternion (needed by ROS2)
    half = yaw * 0.5
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


def main():
    # 1. Initialise ROS2 and create a node
    rclpy.init()
    node = rclpy.create_node('hs_waypoint_follower_nav2pose')

    # 2. Create an ActionClient for the NavigateToPose action
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # start changes

    # --- Weight tracking from HMI status ---
    latest_weight = {'value': None}  # None = N/A / no reading yet

    def parse_weight(status_json: str):
        """Return weight in kg as a float, or None if N/A / unparsable."""
        try:
            data = json.loads(status_json)
            weight_raw = data.get('box', {}).get('weight_raw', '')
        except (json.JSONDecodeError, AttributeError):
            return None
        match = re.search(r'(-?\d+(?:\.\d+)?)\s*kg', weight_raw)
        return float(match.group(1)) if match else None

    def hmi_status_cb(msg: String):
        latest_weight['value'] = parse_weight(msg.data)

    node.create_subscription(String, '/hmi/unified_status', hmi_status_cb, 10)

    def weight_is_positive() -> bool:
        w = latest_weight['value']
        return w is not None and w > 0.0

    def wait_for_weight_positive():
        node.get_logger().info('Waiting for a box to be weighed...')
        while rclpy.ok() and not weight_is_positive():
            rclpy.spin_once(node, timeout_sec=0.2)
        node.get_logger().info(f"Box detected, weight={latest_weight['value']} kg")

    # end

    # --- Function to send a goal and wait for result ---
    def send_and_wait(pose: PoseStamped) -> bool:
        node.get_logger().info('Waiting for Nav2 action server...')
        client.wait_for_server()

        # Update timestamp (required in headers)
        pose.header.stamp = node.get_clock().now().to_msg()

        # Wrap pose in a NavigateToPose goal message
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # Simple feedback callback: prints distance left to target
        def feedback_cb(fb):
            try:
                dist = fb.feedback.distance_remaining
                node.get_logger().info(f'Distance remaining: {dist:.2f} m')
            except Exception:
                pass  # ignore if feedback doesn't have distance

        # Send the goal
        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()

        if not handle or not handle.accepted:
            node.get_logger().error('Goal was rejected!')
            return False

        # Wait until navigation is finished
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()

        if result is None:
            node.get_logger().error('No result returned.')
            return False

        # A finished goal is not always a successful one - check the status
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            node.get_logger().error(f'Goal failed (status {result.status}, '
                                    f'error code {result.result.error_code})')
            return False

        node.get_logger().info('Goal reached successfully!')
        return True

    # --- Hard-coded waypoints for this lab. Edit this section in the code and make it your own ---

    wp1 = make_pose(5.4, 0.7, -1.5708)          # starting position? pointed east - coveyor point c

                                                # pick pose

    wp2 = make_pose(5.4, 0.2, -1.5708)          # coveyor point b
    wp3 = make_pose(5.4, -1.4, -1.5708)         # conveyor point a

                                                # stow pose

    wp4 = make_pose(5.4, -1.4, 0.0)             # conveyor point a rotated "north"
    
    wp6 = make_pose(30.0, -2.0, 0.0)            # west and inline of shelf 
    wp7 = make_pose(30.0, -2.0, -1.5708)        # same point rotated east
    wp8 = make_pose(32.0, -5.45, -1.5708)       # shelf placing position

                                                # place pose
                                                # stow pose

    wp9 = make_pose(32.0, -5.45, 1.5708)        # shelf placing position rotated west
    wp10 = make_pose(32.0, -2.0, 1.5708)        # moved out of shelf area


    # start changes
    
    # 3. Main loop
    try:
        while rclpy.ok():
            # Block here until a box with weight > 0 is present
            wait_for_weight_positive()

            # --- Pickup leg: retry wp1->wp3 until the box is actually picked up ---
            while True:
                send_and_wait(wp1)
                send_and_wait(wp2)
                send_and_wait(wp3)

                rclpy.spin_once(node, timeout_sec=0.2)  # get a fresh weight reading
                if weight_is_positive():
                    node.get_logger().warn('Still weighted after wp3 - retrying pickup leg.')
                    continue
                break

            # --- Delivery leg ---
            send_and_wait(wp4)
            send_and_wait(wp6)
            send_and_wait(wp7)
            send_and_wait(wp8)
            send_and_wait(wp9)
            send_and_wait(wp10)

            # Loop restarts: wait_for_weight_positive() returns immediately if a
            # new box is already waiting, otherwise it blocks until one arrives
            # (or the node is terminated).
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')

    # end    
    # --- Your custom code ends here ---

    # 6. Shutdown node and ROS2
    node.get_logger().info('Navigation sequence complete. Shutting down.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()