#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from collections import deque
import time
import threading
import apriltag




# Tunable parameters
BASE_SPEED = 0.25  # Base forward speed
CURVE_SPEED = 0.20  # Reduced speed for curves
P_GAIN = 0.4  # Proportional gain
D_GAIN = 0.2  # Derivative gain - helps with curves
MAX_STEER = 0.4  # Maximum steering adjustment #
SMOOTHING_STRAIGHT = 3  # Smoothing for straight roads
SMOOTHING_CURVE = 2  # Less smoothing for curves


class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION)

        # Initialize parameters
        self.base_speed = BASE_SPEED
        self.curve_speed = CURVE_SPEED
        self.p_gain = P_GAIN
        self.d_gain = D_GAIN
        self.max_steer = MAX_STEER

        self.red_stop_start_time = None
        self.next_turn_is_right = True

        self.red_cooldown_end_time = 0
        self.turning = False

        # Error tracking for derivative control
        self.prev_error = 0
        self.left_motor_history = deque(maxlen=SMOOTHING_STRAIGHT)
        self.right_motor_history = deque(maxlen=SMOOTHING_STRAIGHT)

        # Setup ROS nodes
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._window = "camera-reader"
        self.bridge = CvBridge()
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        self.sub = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.callback)
        self._publisher = rospy.Publisher(
            wheels_topic, WheelsCmdStamped, queue_size=1)

        self.left_motor = rospy.Publisher("left_motor", Float64, queue_size=1)
        self.right_motor = rospy.Publisher("right_motor", Float64, queue_size=1)

        self.shutting_down = False

        # AprilTag detection
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
        self.SIGN_MAPPING = {
            20: "stop_sign", 24: "stop_sign", 26: "stop_sign",
            31: "stop_sign", 32: "stop_sign", 33: "stop_sign",
            96: "slow_down_sign"
        }
        self.sign_last_seen = None
        self.sign_detected_time = 0
        self.stop_duration = 7  # seconds to stop on stop sign
        self.slow_duration = 4  # seconds to slow on slow sign

        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.shutting_down = True
        self.left_motor.publish(0)
        self.right_motor.publish(0)
        cv2.destroyAllWindows()

    def smooth_motor_value(self, value, history_buffer):
        """Apply smoothing to motor values"""
        history_buffer.append(value)
        return sum(history_buffer) / len(history_buffer)



    def turn_right(self):
        if self.turning:
            return
        self.turning = True
        rospy.loginfo("Turning RIGHT")
        
        # Go forward
        self.publish_motor(0.3, 0.3)
        time.sleep(1.1)

        # Turn right
        self.publish_motor(0.3, -0.3)
        time.sleep(0.37)

        # Go forward again
        self.publish_motor(0.3, 0.3)
        time.sleep(0.3)

        # Stop
        self.publish_motor(0.0, 0.0)
        self.red_cooldown_end_time = time.time() + 1.0  # Ignore red for next 3 seconds
        self.turning = False

        for _ in range(3):
            rospy.sleep(0.05)
            rospy.loginfo("Flushing camera messages...")

    def turn_left(self):

        if self.turning:
            return
        self.turning = True

        rospy.loginfo("Turning LEFT")
        # Go forward longer
        self.publish_motor(0.3, 0.3)
        time.sleep(2.0)

        # Turn left
        self.publish_motor(-0.3, 0.3)
        time.sleep(0.32)

        # Go forward again
        self.publish_motor(0.3, 0.3)
        time.sleep(1.3)
    
        # Stop
        self.publish_motor(0.0, 0.0)
        self.red_cooldown_end_time = time.time() + 1.0  # Ignore red for next 2 seconds
        self.turning = False

        # Flush the camera message queue
        for _ in range(3):
            rospy.sleep(0.05)
            rospy.loginfo("Flushing camera messages...")



    def publish_motor(self, left, right):
        self.left_motor.publish(left)
        self.right_motor.publish(right)

    def callback(self, msg):

        if self.turning or self.shutting_down:
            return

    
        # Process image
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.image = cv2.bilateralFilter(self.image, 9, 75, 75)  # Less aggressive filtering

        # Create a copy for visualization
        vis_image = self.image.copy()

        h, w = self.image.shape[:2]

        # Color space conversions for full image
        luv = cv2.cvtColor(self.image, cv2.COLOR_BGR2LUV)
        hls = cv2.cvtColor(self.image, cv2.COLOR_BGR2HLS)
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # --- Red line detection (full image) ---
        # Red has two ranges in HSV: low (0–10) and high (160–179)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # Create two masks
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # Combine them
        mask_red_full = cv2.bitwise_or(mask_red1, mask_red2)

        # Mask out irrelevant parts (only bottom 25% height and horizontal crop)
        red_roi_y_start = int(h * 0.75)
        red_roi_x_start = w // 2 - w // 4 + 80
        red_roi_x_end = w // 2 + w // 4 - 80

        mask_red_full[:red_roi_y_start, :] = 0
        mask_red_full[:, :red_roi_x_start] = 0
        mask_red_full[:, red_roi_x_end:] = 0

        mask_red = cv2.dilate(mask_red_full, np.ones((3, 3), np.uint8), iterations=1)

        # --- Yellow line detection ---

        lb_yellow = np.array([20, 89, 140])  # Slightly adjusted values
        ub_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lb_yellow, ub_yellow)
        mask_yellow[:int(h * 0.55), :] = 0
        kernel = np.ones((5, 5), np.uint8)
        mask_yellow = cv2.dilate(mask_yellow, kernel, iterations=1)

        # --- white line detection ---

        lb_white = np.array([0, 0, 190])  # Slightly adjusted values
        ub_white = np.array([179, 60, 255])
        mask_white = cv2.inRange(hsv, lb_white, ub_white)
        mask_white[:int(h * 0.55), :] = 0
        mask_white = cv2.dilate(mask_white, kernel, iterations=1)


        # Find contours (for visualization)
        yellow_contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        white_contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours(vis_image, yellow_contours, -1, (0, 255, 255), 2)
        cv2.drawContours(vis_image, white_contours, -1, (255, 255, 255), 2)
        cv2.drawContours(vis_image, red_contours, -1, (0, 0, 255), 2)

        # Initialize detection flags
        left_line_detected = len(yellow_contours) > 0
        right_line_detected = len(white_contours) > 0
        red_line_detected = len(red_contours) > 0

        # Slices for curve detection
        num_slices = 3
        slice_height = int(h * 0.35 / num_slices)
        start_y = int(h * 0.55)

        yellow_x_points = []
        white_x_points = []
        red_x_points = []
        y_points = []

        for i in range(num_slices):
            y = start_y + i * slice_height + slice_height // 2
            y_points.append(y)

            # Yellow slice
            slice_yellow = mask_yellow[y - 5:y + 5, :]
            yellow_indices = np.where(slice_yellow > 0)[1]
            if len(yellow_indices) > 0:
                yellow_x = int(np.mean(yellow_indices))
                yellow_x_points.append(yellow_x)
                cv2.circle(vis_image, (yellow_x, y), 5, (0, 255, 255), -1)

            # White slice
            slice_white = mask_white[y - 5:y + 5, :]
            white_indices = np.where(slice_white > 0)[1]
            if len(white_indices) > 0:
                white_x = int(np.mean(white_indices))
                white_x_points.append(white_x)
                cv2.circle(vis_image, (white_x, y), 5, (255, 255, 255), -1)

            # Red slice
            slice_red = mask_red[y - 5:y + 5, :]
            red_indices = np.where(slice_red > 0)[1]
            if len(red_indices) > 0:
                red_x = int(np.mean(red_indices))
                red_x_points.append(red_x)
                cv2.circle(vis_image, (red_x, y), 5, (0, 0, 255), -1)

        # Curve detection logic
        is_curve = False
        curve_direction = 0

        if len(yellow_x_points) >= 2:
            yellow_diff = yellow_x_points[-1] - yellow_x_points[0]
            if abs(yellow_diff) > 20:
                is_curve = True
                curve_direction += np.sign(yellow_diff)

        if len(white_x_points) >= 2:
            white_diff = white_x_points[-1] - white_x_points[0]
            if abs(white_diff) > 20:
                is_curve = True
                curve_direction += np.sign(white_diff)


        current_time = time.time()
        if current_time < self.red_cooldown_end_time:
            red_x_points = []
            rospy.loginfo("Red detection cooldown active")
        

        # --- RED STOP LOGIC ---
        if len(red_x_points) > 0:
            if self.red_stop_start_time is None:
                self.red_stop_start_time = time.time()

            elapsed = time.time() - self.red_stop_start_time
            if elapsed < 0.8: #0.3
                # Stop motors for 2 seconds
                left_motor = 0.0
                right_motor = 0.0

                # Show stop message
                cv2.putText(vis_image, "RED LINE DETECTED - STOPPING", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # Stop motors
                self.left_motor.publish(left_motor)
                self.right_motor.publish(right_motor)

                cv2.imshow(self._window, vis_image)
                cv2.waitKey(1)
                return  # Wait until stop time completes
            else:
                # Perform the turn
                if self.next_turn_is_right:
                    threading.Thread(target=self.turn_right).start()
                else:
                    threading.Thread(target=self.turn_left).start()

                # Alternate for next red
                self.next_turn_is_right = not self.next_turn_is_right

                # Reset timer
                self.red_stop_start_time = None
                return  # Important to skip rest of callback this frame
        else:
            # No red, reset timer
            self.red_stop_start_time = None

        # Center and error calculation
        if left_line_detected and right_line_detected and len(yellow_x_points) > 0 and len(white_x_points) > 0:
            center_position = (yellow_x_points[-1] + white_x_points[-1]) / 2
            ideal_center = w / 2
            error = ideal_center - center_position
        elif left_line_detected and len(yellow_x_points) > 0:
            error = w / 2 - (yellow_x_points[-1] + 160)
        elif right_line_detected and len(white_x_points) > 0:
            error = w / 2 - (white_x_points[-1] - 160)
        else:
            error = self.prev_error

        error = np.clip(error / (w / 2), -1, 1)
        error_diff = error - self.prev_error
        self.prev_error = error

        # PD control
        steering = self.p_gain * error + self.d_gain * error_diff
        steering = np.clip(steering, -self.max_steer, self.max_steer)

        current_speed = self.curve_speed if is_curve else self.base_speed

        left_motor = current_speed - steering
        right_motor = current_speed + steering

        if is_curve and abs(steering) > 0.3:
            if steering > 0:
                right_motor *= 1.3
            else:
                left_motor *= 1.3

        line_pixels = np.count_nonzero(mask_yellow) + np.count_nonzero(mask_white)
        if line_pixels < 500:
            left_motor = -0.2
            right_motor = -0.3

        smoothing_amount = SMOOTHING_CURVE if is_curve else SMOOTHING_STRAIGHT
        self.left_motor_history = deque(self.left_motor_history, maxlen=smoothing_amount)
        self.right_motor_history = deque(self.right_motor_history, maxlen=smoothing_amount)

        left_motor = self.smooth_motor_value(left_motor, self.left_motor_history)
        right_motor = self.smooth_motor_value(right_motor, self.right_motor_history)

        left_motor = np.clip(left_motor, -1.0, 1.0)
        right_motor = np.clip(right_motor, -1.0, 1.0)

        if not self.shutting_down:
            self.left_motor.publish(left_motor)
            self.right_motor.publish(right_motor)


        # ----- AprilTag SIGN DETECTION -----
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        sign_detected = None
        closest_sign = None
        max_area = 0

        for detection in detections:
            tag_id = detection.tag_id
            sign_type = self.SIGN_MAPPING.get(tag_id)
            if sign_type:
                tag_area = cv2.contourArea(detection.corners.astype(np.int32))
                if tag_area > 2000 and tag_area > max_area:
                    max_area = tag_area
                    closest_sign = detection
                    sign_detected = sign_type

        if closest_sign:
            for i in range(4):
                pt1 = tuple(map(int, closest_sign.corners[i]))
                pt2 = tuple(map(int, closest_sign.corners[(i + 1) % 4]))
                cv2.line(vis_image, pt1, pt2, (255, 0, 0), 2)  # Blue bounding box
            center = tuple(map(int, closest_sign.center))
            label = f"{sign_detected.upper()}"
            cv2.putText(vis_image, label, (center[0] - 40, center[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # ----- FINAL STOP / SLOW LOGIC -----
        if sign_detected == "stop_sign":
            rospy.loginfo("STOP sign detected up close. Stopping permanently.")
            self.publish_motor(0.0, 0.0)
            cv2.putText(vis_image, "STOP SIGN DETECTED - TERMINATING", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow(self._window, vis_image)
            cv2.waitKey(1)
            rospy.signal_shutdown("STOP sign triggered shutdown.")
            return

        elif sign_detected == "slow_down_sign":
            if self.sign_last_seen != "slow_down_sign" or (time.time() - self.sign_detected_time > 10):
                rospy.loginfo("SLOW DOWN sign detected - Reducing speed")
                self.sign_detected_time = time.time()
                self.sign_last_seen = "slow_down_sign"

            if time.time() - self.sign_detected_time <= 5:
                self.base_speed = BASE_SPEED * 0.3
                self.curve_speed = CURVE_SPEED * 0.3
                cv2.putText(vis_image, "SLOW DOWN ACTIVE", (5, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            else:
                self.base_speed = BASE_SPEED
                self.curve_speed = CURVE_SPEED


        # Display info including red line detection
        cv2.putText(vis_image, f"Curve: {is_curve}, Dir: {curve_direction}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(vis_image, f"Error: {error:.2f}, Steer: {steering:.2f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(vis_image, f"Red line detected: {red_line_detected}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        if not self.turning:
            cv2.imshow(self._window, vis_image)
            cv2.waitKey(1)

if __name__ == '__main__':
    node = CameraReaderNode(node_name='camera_reader_node')
    rospy.spin()