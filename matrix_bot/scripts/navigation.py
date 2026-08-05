#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool

class SquarePatrolService(Node):
    def __init__(self):
        super().__init__('square_patrol_service')
        
        # 1. Create the Service Server to start/stop patrol
        self.srv = self.create_service(SetBool, 'toggle_patrol', self.toggle_patrol_callback)
        
        # 2. Create the Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 3. Define the 2m x 2m square waypoints
        self.waypoints = [
            (2.0, 0.0),   # Point 1: Drive forward 2 meters
            (2.0, 2.0),   # Point 2: Turn left and drive 2 meters
            (0.0, 2.0),   # Point 3: Turn left and drive 2 meters
            (0.0, 0.0)    # Point 4: Return to origin
        ]
        
        self.current_index = 0
        self.is_patrolling = False
        self.goal_handle = None

        self.get_logger().info("Square Patrol Node Ready. Service /toggle_patrol is active.")

    def toggle_patrol_callback(self, request, response):
        """Service callback to handle incoming start/stop requests"""
        if request.data:  # Signal to START
            if not self.is_patrolling:
                self.is_patrolling = True
                self.get_logger().info("Square patrol STARTED! Traveling to Point 1...")
                self.send_next_goal()
                response.success = True
                response.message = "Continuous square patrol initiated."
            else:
                response.success = True
                response.message = "Patrol is already running."
        else:            # Signal to STOP
            if self.is_patrolling:
                self.is_patrolling = False
                self.get_logger().info("Square patrol STOPPED! Canceling active goal...")
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                response.success = True
                response.message = "Patrol successfully stopped."
            else:
                response.success = True
                response.message = "Patrol was already idle."
        
        return response

    def send_next_goal(self):
        """Sends the target coordinate to the Nav2 Action Server"""
        if not self.is_patrolling:
            return

        # FIXED: Correct API method 'wait_for_server' instead of 'wait_for_action_server'
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 '/navigate_to_pose' server not found! Is Nav2 running?")
            self.is_patrolling = False
            return

        x, y = self.waypoints[self.current_index]
        self.get_logger().info(f"Targeting Corner {self.current_index + 1}: X={x}, Y={y}")

        # Package the Nav2 Goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Default neutral orientation

        # Send goal asynchronously
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Checks if Nav2 accepted or rejected the waypoint request"""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Waypoint rejected by Nav2 planner.")
            return

        # Monitor progress until the robot physically arrives
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Fires when the robot reaches the corner or fails trying"""
        result = future.result()
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Successfully arrived at Corner {self.current_index + 1}!")
        else:
            self.get_logger().warn(f"Missed Corner {self.current_index + 1}. Moving to next target.")

        # Cycle continuously to the next waypoint if patrol is still active
        if self.is_patrolling:
            self.current_index = (self.current_index + 1) % len(self.waypoints)
            self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = SquarePatrolService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()