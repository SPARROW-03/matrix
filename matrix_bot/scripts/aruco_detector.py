#!/usr/bin/env python3
"""
aruco_detector.py

Computes direct lateral offset (dy) and forward distance (dx)
relative to camera/bot centerline without TF offset drift.
"""

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


def normalize_angle(angle):
    """Wraps angle into [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quat(yaw):
    """Converts planar yaw to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class ArucoDockDetector(Node):
    def __init__(self):
        super().__init__('aruco_dock_detector')

        self.declare_parameter('dock_marker_id', 0)
        self.declare_parameter('marker_size', 0.12)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('aruco_dict', 'DICT_5X5_50')
        self.declare_parameter('standoff', 0.05)

        self.dock_marker_id = self.get_parameter('dock_marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.base_frame = self.get_parameter('base_frame').value
        self.standoff = self.get_parameter('standoff').value

        dict_name = self.get_parameter('aruco_dict').value
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        self.create_subscription(
            CameraInfo,
            '/camera_sensor/camera_info',
            self.camera_info_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_cb,
            qos_profile_sensor_data
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)
        self.get_logger().info('Aruco Dock Detector running.')

    def camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics loaded.')

    def image_cb(self, msg: Image):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img_h, img_w = gray.shape[:2]

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return

        ids = ids.flatten()
        if self.dock_marker_id not in ids:
            return

        idx = int(np.where(ids == self.dock_marker_id)[0][0])
        marker_corners = corners[idx].reshape((4, 2))

        # 1. Image Centroid
        tag_center_u = float(np.mean(marker_corners[:, 0]))
        tag_center_v = float(np.mean(marker_corners[:, 1]))

        # Image Optical Center
        cx = float(self.camera_matrix[0, 2]) if self.camera_matrix[0, 2] > 0 else (img_w / 2.0)
        cy = float(self.camera_matrix[1, 2]) if self.camera_matrix[1, 2] > 0 else (img_h / 2.0)
        fx = float(self.camera_matrix[0, 0])

        # 2. ArUco 3D solvePnP
        s = self.marker_size / 2.0
        obj_points = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0]
        ], dtype=np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            marker_corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            return

        # Forward depth in meters (Optical Z)
        depth_z = float(tvec[2][0] if tvec.ndim == 2 else tvec[2])

        # 3. Direct Ray-Projection for Lateral Position (dy)
        # Offset in pixels relative to center
        # u < cx (tag on LEFT side of image)  -> dy > 0 (+Y)
        # u > cx (tag on RIGHT side of image) -> dy < 0 (-Y)
        # u == cx (tag in CENTER)             -> dy = 0.000 m
        pixel_offset_x = tag_center_u - cx
        dx = depth_z
        dy = -(pixel_offset_x * depth_z) / fx

        # 4. Marker Heading Error (dyaw)
        R_cv, _ = cv2.Rodrigues(rvec)
        marker_normal_opt = R_cv[:, 2]
        norm_x = marker_normal_opt[2]
        norm_y = -marker_normal_opt[0]
        dyaw = normalize_angle(math.atan2(-norm_y, -norm_x))

        # 5. Publish PoseStamped in base_link
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.base_frame
        pose_msg.pose.position.x = float(dx)
        pose_msg.pose.position.y = float(dy)
        pose_msg.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(dyaw)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        # 6. Terminal Readout
        dist_err = dx - self.standoff
        dyaw_deg = math.degrees(dyaw)

    #    print("\n" + "=" * 60)
    #    print(f" [ArUco ID {self.dock_marker_id}] DOCK RELATIVE POSE")
    #    print("-" * 60)
    #    print(f" Image Center (cx)        : {cx:.1f} px | Frame Width: {img_w} px")
    #    print(f" Tag Centroid (u)         : {tag_center_u:.1f} px")
    #    print(f" Pixel Offset (u - cx)    : {pixel_offset_x:+.1f} px")
    #    print(f" Forward Distance (dx)    : {dx:+.4f} m (Err: {dist_err:+.4f} m)")
    #    print(f" Lateral Offset   (dy)    : {dy:+.4f} m")
    #    print(f" Heading Error    (dyaw)  : {dyaw:+.4f} rad ({dyaw_deg:+.2f}°)")
    #    print(f" Observation Vector       : [{dx:.3f}, {dy:.3f}, {math.sin(dyaw):.3f}, {math.cos(dyaw):.3f}]")
    #    print("=" * 60)

def main():
    rclpy.init()
    node = ArucoDockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()