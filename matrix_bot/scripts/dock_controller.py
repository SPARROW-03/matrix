#!/usr/bin/env python3
"""
Action server that handles both RL Docking and Odometry-based Undocking:
1. DockToStation:
   - SEARCH: Rotates to bring the ArUco marker into FOV.
   - RL_DOCK: Continuous ONNX policy inference until robot settles within explicit threshold bounds:
       * dx in [0.25m, 0.35m]
       * dy in [-0.05m, +0.05m]
       * dyaw in [-10.0 deg, +10.0 deg]
2. UndockFromStation:
   - Reverses a set distance using /odometry/filtered feedback.
"""
import os
import math
import time
import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

from matrix_interfaces.action import DockToStation, UndockFromStation



def quat_to_yaw(q):
    """Converts a quaternion into planar yaw (radians)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Wraps angle into [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class DockController(Node):

    SEARCH, RL_DOCK = range(2)

    def __init__(self):
        super().__init__('dock_controller')
        self.cb_group = ReentrantCallbackGroup()

        # ==========================================
        # Explicit Threshold Parameters
        # ==========================================
        pkg= get_package_share_directory('matrix_bot')
        model_path = os.path.join(pkg, 'Model', 'docking_policy.onnx')
        self.declare_parameter('onnx_model_path', model_path)
        
        # Explicit Range Bounds
        self.declare_parameter('dx_min', 0.10)                    # Minimum forward distance (m)
        self.declare_parameter('dx_max', 0.35)                    # Maximum forward distance (m)
        self.declare_parameter('dy_strict_limit', 0.08)           # Strict lateral tolerance: [-0.08m, +0.08m]
        self.declare_parameter('dyaw_limit_deg', 8.0)            # Heading tolerance: [-10.0 deg, +10.0 deg]
        self.declare_parameter('settle_cycles_required', 5)       # Consecutive control cycles required

        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('marker_lost_timeout', 0.6)
        self.declare_parameter('search_angular_speed', 0.3)
        self.declare_parameter('search_timeout', 15.0)

        # ==========================================
        # Parameters - Undocking
        # ==========================================
        self.declare_parameter('undock_backup_distance', 0.50)
        self.declare_parameter('undock_speed', 0.10)
        self.declare_parameter('undock_tolerance', 0.01)

        # Cache parameter values
        model_path = self.get_parameter('onnx_model_path').value
        self.dx_min = self.get_parameter('dx_min').value
        self.dx_max = self.get_parameter('dx_max').value
        self.dy_limit = self.get_parameter('dy_strict_limit').value
        self.dyaw_limit_deg = self.get_parameter('dyaw_limit_deg').value
        self.dyaw_limit_rad = math.radians(self.dyaw_limit_deg)
        self.settle_required = self.get_parameter('settle_cycles_required').value

        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value
        self.lost_timeout = self.get_parameter('marker_lost_timeout').value
        self.search_speed = self.get_parameter('search_angular_speed').value
        self.search_timeout = self.get_parameter('search_timeout').value

        self.undock_dist = self.get_parameter('undock_backup_distance').value
        self.undock_spd = self.get_parameter('undock_speed').value
        self.undock_tol = self.get_parameter('undock_tolerance').value

        # Load ONNX Policy
        try:
            self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
            self.input_name = self.session.get_inputs()[0].name
            self.get_logger().info(f'Loaded ONNX RL policy from: {model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load ONNX model: {e}')
            raise e

        # State Variables
        self.latest_pose = None
        self.last_seen_time = None
        self.current_odom = None
        self.lin_v = 0.0
        self.ang_v = 0.0

        # Subscriptions and Publishers
        self.create_subscription(
            PoseStamped, '/detected_dock_pose', self.pose_cb, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_cb, 10, callback_group=self.cb_group
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Action Servers
        self._dock_server = ActionServer(
            self,
            DockToStation,
            'dock_to_station',
            execute_callback=self.execute_dock_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group
        )

        self._undock_server = ActionServer(
            self,
            UndockFromStation,
            'undock_from_station',
            execute_callback=self.execute_undock_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group
        )

        self.get_logger().info('DockController initialized with exact threshold bounds.')

    def goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn('Goal cancel requested.')
        self.stop_robot()
        return CancelResponse.ACCEPT

    def pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg
        self.last_seen_time = time.time()

    def odom_cb(self, msg: Odometry):
        self.current_odom = msg
        self.lin_v = msg.twist.twist.linear.x
        self.ang_v = msg.twist.twist.angular.z

    def marker_visible(self):
        return (self.last_seen_time is not None and
                (time.time() - self.last_seen_time) < self.lost_timeout)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ==========================================
    # Action Execution: DockToStation (RL Policy)
    # ==========================================
    def execute_dock_cb(self, goal_handle):
        self.get_logger().info('DockToStation goal accepted. Starting sequence...')
        state = self.SEARCH
        search_start = time.time()
        settle_count = 0

        rate = self.create_rate(50)  # 50 Hz control loop
        result = DockToStation.Result()
        feedback = DockToStation.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = 'Docking canceled by user.'
                return result

            # Perception Check
            if not self.marker_visible():
                if state != self.SEARCH:
                    self.get_logger().warn('Tag lost. Switching to SEARCH mode.')
                    search_start = time.time()
                    settle_count = 0
                state = self.SEARCH
            else:
                state = self.RL_DOCK

            # 1. SEARCH State
            if state == self.SEARCH:
                if (time.time() - search_start) > self.search_timeout:
                    self.stop_robot()
                    result.success = False
                    result.message = 'Docking aborted: Marker not found within timeout.'
                    goal_handle.abort()
                    return result

                feedback.status = 'SEARCHING'
                goal_handle.publish_feedback(feedback)

                twist = Twist()
                twist.angular.z = self.search_speed
                self.cmd_pub.publish(twist)

            # 2. RL_DOCK State
            elif state == self.RL_DOCK:
                dx = self.latest_pose.pose.position.x
                dy = self.latest_pose.pose.position.y
                dyaw = normalize_angle(quat_to_yaw(self.latest_pose.pose.orientation))
                dyaw_deg = math.degrees(dyaw)

                # Direct evaluation of requested threshold boundaries
                inside_dx_bounds   = (self.dx_min <= dx <= self.dx_max)
                inside_dy_bounds   = (-self.dy_limit <= dy <= self.dy_limit)
                inside_dyaw_bounds = (-self.dyaw_limit_rad <= dyaw <= self.dyaw_limit_rad)

                if inside_dx_bounds and inside_dy_bounds and inside_dyaw_bounds:
                    settle_count += 1
                else:
                    settle_count = 0

                # Trigger SUCCESS once robot settles in bounds
                if settle_count >= self.settle_required:
                    self.stop_robot()
                    result.success = True
                    result.message = (f'Docked successfully: dx={dx:.3f}m [{self.dx_min}m - {self.dx_max}m], '
                                      f'dy={dy:.3f}m [±{self.dy_limit}m], '
                                      f'yaw={dyaw_deg:.1f}° [±{self.dyaw_limit_deg:.1f}°]')
                    self.get_logger().info(result.message)
                    goal_handle.succeed()
                    return result

                feedback.status = (f'RL_DOCKING: dx={dx:.2f}m [{self.dx_min}-{self.dx_max}], '
                                   f'dy={dy:.2f}m [±{self.dy_limit}], '
                                   f'yaw={dyaw_deg:.1f}° [±{self.dyaw_limit_deg:.0f}°] '
                                   f'(Settle: {settle_count}/{self.settle_required})')
                goal_handle.publish_feedback(feedback)

                # Observation Vector: [dx, dy, sin(dyaw), cos(dyaw), lin_v, ang_v]
                obs = np.array([
                    dx,
                    dy,
                    np.sin(dyaw),
                    np.cos(dyaw),
                    self.lin_v,
                    self.ang_v
                ], dtype=np.float32).reshape(1, 6)

                raw_action = self.session.run(None, {self.input_name: obs})[0][0]

                lin_cmd = float(np.clip(raw_action[0], -1.0, 1.0)) * self.max_lin
                ang_cmd = float(np.clip(raw_action[1], -1.0, 1.0)) * self.max_ang

                twist = Twist()
                twist.linear.x = lin_cmd
                twist.angular.z = ang_cmd
                self.cmd_pub.publish(twist)

            rate.sleep()

        self.stop_robot()
        result.success = False
        result.message = 'Node shutting down.'
        return result

    # ==========================================
    # Action Execution: UndockFromStation
    # ==========================================
    def execute_undock_cb(self, goal_handle):
        self.get_logger().info('UndockFromStation goal accepted. Backing out...')
        result = UndockFromStation.Result()
        rate = self.create_rate(20)

        start_wait = time.time()
        while self.current_odom is None and (time.time() - start_wait) < 3.0:
            rate.sleep()

        if self.current_odom is None:
            self.get_logger().error('Undock failed: No odometry data received.')
            goal_handle.abort()
            result.success = False
            result.message = 'Missing odometry data.'
            return result

        start_x = self.current_odom.pose.pose.position.x
        start_y = self.current_odom.pose.pose.position.y

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = 'Undocking canceled by user.'
                return result

            curr_x = self.current_odom.pose.pose.position.x
            curr_y = self.current_odom.pose.pose.position.y

            traveled = math.hypot(curr_x - start_x, curr_y - start_y)
            remaining = self.undock_dist - traveled

            feedback = UndockFromStation.Feedback()
            feedback.status = f'BACKING_OUT: {traveled:.2f}m / {self.undock_dist:.2f}m'
            goal_handle.publish_feedback(feedback)

            if remaining <= self.undock_tol:
                self.stop_robot()
                self.get_logger().info(f'Undocked successfully ({traveled:.3f} m).')
                result.success = True
                result.message = f'Undocked successfully ({traveled:.2f} m).'
                goal_handle.succeed()
                return result

            speed_factor = min(1.0, max(0.3, remaining / 0.10))
            twist = Twist()
            twist.linear.x = -abs(self.undock_spd) * speed_factor
            twist.angular.z = 0.0

            self.cmd_pub.publish(twist)
            rate.sleep()

        self.stop_robot()
        result.success = False
        result.message = 'Node shutting down.'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DockController()

    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()