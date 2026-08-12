#!/usr/bin/env python3
import os
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory
from matrix_interfaces.action import NavigateToLocation
from matrix_interfaces.srv import GetLocation

class LocationNavigation(Node):
    def __init__(self):
        super().__init__('location_navigation')
        path = os.path.join(get_package_share_directory('matrix_bot'), 'config', 'location.yaml')
        with open(path, 'r') as f:
            self.locations = yaml.safe_load(f)['locations']
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.location_action_client = ActionClient(self, NavigateToLocation, '/navigate_to_location')
        self.current_nav_goal = None
        self.last_feedback_time = self.get_clock().now()
        self.location_service = self.create_service(GetLocation, '/get_location', self.get_location)
        self.server = ActionServer(self, NavigateToLocation, '/navigate_to_location', self.execute, cancel_callback=self.cancel_callback)
        self.get_logger().info('Location navigation ready')

    def cancel_callback(self, cancel_request):
        self.get_logger().info('Cancel request received')
        if self.current_nav_goal is not None:
            self.current_nav_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def get_location(self, request, response):
        location = request.location
        if location not in self.locations:
            response.success = False
            response.message = f'Unknown location: {location}'
            return response
        pose = self.locations[location]
        response.success = True
        response.x = pose['x']
        response.y = pose['y']
        response.yaw = pose['yaw']
        response.message = f'Location {location} found'
        return response

    async def execute(self, goal_handle):
        location = goal_handle.request.location
        if location not in self.locations:
            goal_handle.abort()
            result = NavigateToLocation.Result()
            result.success = False
            result.message = f'Unknown location: {location}'
            return result
        pose = self.locations[location]
        self.get_logger().info(f'Navigating to location {location}')
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result = NavigateToLocation.Result()
            result.success = False
            result.message = 'Nav2 not available'
            return result
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose['x']
        goal.pose.pose.position.y = pose['y']
        yaw = pose['yaw']
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        nav_goal = await self.nav_client.send_goal_async(
            goal,
            feedback_callback=lambda msg: self.feedback(msg, goal_handle)
        )
        if not nav_goal.accepted:
            goal_handle.abort()
            result = NavigateToLocation.Result()
            result.success = False
            result.message = 'Nav2 rejected goal'
            return result
        self.current_nav_goal = nav_goal
        self.get_logger().info('Nav2 goal accepted')
        nav_result = await nav_goal.get_result_async()
        self.current_nav_goal = None
        result = NavigateToLocation.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = 'Navigation canceled'
            return result
        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
            result.success = True
            result.message = f'Reached location {location}'
        else:
            goal_handle.abort()
            result.success = False
            result.message = f'Failed to reach location {location}'
        return result

    def feedback(self, msg, goal_handle):
        now = self.get_clock().now()
        if (now - self.last_feedback_time).nanoseconds < 5e9:
            return
        self.last_feedback_time = now
        feedback = NavigateToLocation.Feedback()
        feedback.distance_remaining = msg.feedback.distance_remaining
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} m')

def main():
    rclpy.init()
    node = LocationNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()