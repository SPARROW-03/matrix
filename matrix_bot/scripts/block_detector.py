#!/usr/bin/env python3
"""
marker_follower.py

Sequentially search for, approach, and center on printed number signs
(digits 1..6) using camera feed + multi-scale template matching + cmd_vel.

Why template matching instead of OCR:
    Tesseract could not reliably read the stylized digit font used in
    this simulation (tested directly against captured frames -- it
    misread '1' as '<' / '~' even on a clean isolated crop). Since this
    is a controlled sim with the *same* rendered texture every time,
    template matching against a captured reference image of each digit
    is far more reliable than general-purpose OCR here.

Setup required BEFORE running this node:
    1. Run capture_frame.py while facing each digit sign head-on, once
       per digit (produces raw_digit_1.png ... raw_digit_6.png).
    2. Run crop_template.py on each to draw a tight box around just the
       digit, saving into templates/digit_1.png ... templates/digit_6.png
    3. Set TEMPLATE_DIR below to point at that templates/ folder.

Behavior:
  - SEARCHING : target digit not visible yet -> rotate in place to scan
  - TRACKING  : target digit visible -> yaw-center it + move forward toward it
  - Advance to next digit when either:
        a) matched region area >= CLOSE_ENOUGH_AREA for CONFIRM_FRAMES
           consecutive frames in a row ("close enough, confirmed"), or
        b) digit was being tracked and then disappears from frame for
           LOST_FRAMES_LIMIT consecutive frames (interpreted as
           "passed / close enough to lose FOV")

Fix vs. earlier version:
    Template matching picks whichever (scale, location) combo scores
    highest on a *single* frame. That best-scoring scale can jump around
    frame to frame -- especially at a distance, where scores across
    scales are close together -- which sometimes made the matched box
    appear briefly "large" even though the robot hadn't moved. That let
    a single lucky frame satisfy CLOSE_ENOUGH_AREA and advance the
    target instantly. Fix: require the area to stay >= CLOSE_ENOUGH_AREA
    for CONFIRM_FRAMES consecutive frames (debounced, same idea as the
    existing LOST_FRAMES_LIMIT debounce for "lost") before advancing.
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2


# ----------------------------- Parameters -----------------------------

IMAGE_TOPIC   = '/camera_sensor/image_raw'   # <-- change to match your camera plugin's topic
CMD_VEL_TOPIC = '/cmd_vel'

TEMPLATE_DIR  = os.path.expanduser('~/dev_ws/templates')  # <-- folder containing digit_1.png ... digit_6.png

FIRST_TARGET_ID = 1
LAST_TARGET_ID  = 6

# Multi-scale search: try the template resized across this range (relative
# to its original captured size) to handle the sign appearing larger/smaller
# as the robot approaches or is further away.
SCALE_RANGE   = np.linspace(0.3, 1.6, 14)   # fraction of original template size to try
MATCH_METHOD  = cv2.TM_CCOEFF_NORMED
MATCH_THRESHOLD = 0.80   # 0-1 normalized correlation; TUNE this by testing (see notes at bottom)

# "Close enough" -- based on matched region pixel area at the best-scoring scale.
# TUNE by testing: drive the robot manually toward a sign, print area values,
# and pick a threshold reached shortly before the robot would bump it.
CLOSE_ENOUGH_AREA = 80000   # pixels^2

# Debounce: area must be >= CLOSE_ENOUGH_AREA for this many CONSECUTIVE
# frames before we trust it and advance to the next target. Prevents a
# single noisy/lucky scale match from ending tracking prematurely.
CONFIRM_FRAMES = 5

# --- Control gains ---
KP_ANGULAR         = 0.005
MAX_ANGULAR         = 0.6
SEARCH_ANGULAR      = -0.35

KP_LINEAR            = 0.4
MAX_LINEAR           = 0.25
CENTER_DEADBAND_PX  = 15

LOST_FRAMES_LIMIT   = 8   # consecutive missed frames (while tracking) before treated as "passed"

# ------------------------------------------------------------------------


class MarkerFollower(Node):
    def __init__(self):
        super().__init__('marker_follower')

        self.bridge = CvBridge()
        self.templates = self.load_templates()

        self.current_target = FIRST_TARGET_ID
        self.tracking = False
        self.lost_frame_count = 0
        self.close_frame_count = 0
        self.mission_done = False

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Image, IMAGE_TOPIC, self.image_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'MarkerFollower (template matching) started. '
            f'Loaded {len(self.templates)} templates. First target: digit {self.current_target}'
        )

    # ---------------------------------------------------------------

    def load_templates(self):
        templates = {}
        for digit in range(FIRST_TARGET_ID, LAST_TARGET_ID + 1):
            path = os.path.join(TEMPLATE_DIR, f'digit_{digit}.png')
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                self.get_logger().warn(f'Could not load template for digit {digit} at {path}')
                continue
            templates[digit] = img
        return templates

    # ---------------------------------------------------------------

    def image_cb(self, msg: Image):
        if self.mission_done:
            self.stop_robot()
            return

        template = self.templates.get(self.current_target)
        if template is None:
            self.get_logger().error(f'No template loaded for digit {self.current_target}, cannot proceed.')
            self.stop_robot()
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img_w = frame.shape[1]

        detection = self.match_template_multiscale(gray, template)

        if detection is not None:
            self.lost_frame_count = 0
            self.tracking = True
            self.handle_tracking(detection, img_w)
        else:
            # Detection dropped this frame -> also reset the "close enough"
            # streak, since we no longer have a confirmed area reading.
            self.close_frame_count = 0

            if self.tracking:
                self.lost_frame_count += 1
                if self.lost_frame_count >= LOST_FRAMES_LIMIT:
                    self.get_logger().info(
                        f'Digit {self.current_target} lost from view after tracking -> '
                        f'treating as reached, advancing.'
                    )
                    self.advance_target()
                else:
                    self.publish_cmd(linear=0.05, angular=0.0)
            else:
                self.handle_searching()

    # ---------------------------------------------------------------

    def match_template_multiscale(self, gray_frame, template):
        """
        Slide `template` across `gray_frame` at multiple scales, return the
        best match's bounding box (x, y, w, h) if it clears MATCH_THRESHOLD,
        else None.
        """
        best_score = -1.0
        best_box = None
        th_orig, tw_orig = template.shape[:2]

        for scale in SCALE_RANGE:
            tw = int(tw_orig * scale)
            th = int(th_orig * scale)
            if tw < 8 or th < 8:
                continue
            if tw > gray_frame.shape[1] or th > gray_frame.shape[0]:
                continue

            resized_template = cv2.resize(template, (tw, th), interpolation=cv2.INTER_AREA)
            result = cv2.matchTemplate(gray_frame, resized_template, MATCH_METHOD)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)

            if max_val > best_score:
                best_score = max_val
                best_box = (max_loc[0], max_loc[1], tw, th)

        if best_score >= MATCH_THRESHOLD:
            return best_box
        return None

    # ---------------------------------------------------------------

    def handle_searching(self):
        self.publish_cmd(linear=0.0, angular=SEARCH_ANGULAR)

    # ---------------------------------------------------------------

    def handle_tracking(self, bbox, img_w):
        x, y, w, h = bbox
        cx = x + w / 2.0
        img_center_x = img_w / 2.0
        px_offset = img_center_x - cx

        angular = 0.0
        if abs(px_offset) > CENTER_DEADBAND_PX:
            angular = KP_ANGULAR * px_offset
            angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

        area = w * h
        close_enough = area >= CLOSE_ENOUGH_AREA

        # Debounce: only count this as "close enough" once the area has
        # stayed above threshold for CONFIRM_FRAMES consecutive frames.
        # A single frame where the matcher happens to lock onto a larger
        # scale (without the robot actually being close) won't trigger
        # advance_target() anymore.
        if close_enough:
            self.close_frame_count += 1
        else:
            self.close_frame_count = 0

        self.get_logger().debug(
            f'digit={self.current_target} area={area} '
            f'close_streak={self.close_frame_count}/{CONFIRM_FRAMES} '
            f'px_offset={px_offset:.1f}'
        )

        proxy_error = max(0.0, (CLOSE_ENOUGH_AREA * 3 - area))
        linear = min(MAX_LINEAR, KP_LINEAR * proxy_error / (CLOSE_ENOUGH_AREA * 3))

        if self.close_frame_count >= CONFIRM_FRAMES:
            self.get_logger().info(
                f'Digit {self.current_target} reached (area={area}, '
                f'confirmed over {CONFIRM_FRAMES} frames). Advancing.'
            )
            self.advance_target()
            return

        if abs(px_offset) > CENTER_DEADBAND_PX * 3:
            linear *= 0.4

        self.publish_cmd(linear=linear, angular=angular)

    # ---------------------------------------------------------------

    def advance_target(self):
        self.tracking = False
        self.lost_frame_count = 0
        self.close_frame_count = 0
        self.stop_robot()

        if self.current_target >= LAST_TARGET_ID:
            self.get_logger().info('All targets (1-6) reached. Mission complete.')
            self.mission_done = True
            return

        self.current_target += 1
        self.get_logger().info(f'Now searching for digit {self.current_target}')

    # ---------------------------------------------------------------

    def publish_cmd(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()