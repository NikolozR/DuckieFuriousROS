#!/usr/bin/env python3

import os
import rospy
import cv2
import apriltag
import numpy as np
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String


class TrafficSignNode(DTROS):
    """
    ROS Node for traffic sign detection and response using AprilTags
    """
    
    # Their mappings:
    road_signs = {
        20: "Stop", 24: "Stop", 26: "Stop", 31: "Stop", 32: "Stop", 33: "Stop",
        96: "Slow Down",
        125: "Yield"
    }

    # Our mappings:
    # AprilTag ID to sign type mapping (focused on stop and slow down only)
    SIGN_MAPPING = {
        # Stop signs (multiple IDs for different orientations/versions)
        20: "stop_sign",
        24: "stop_sign", 
        26: "stop_sign",
        31: "stop_sign",
        32: "stop_sign",
        33: "stop_sign",
        
        # Slow down sign
        96: "slow_down_sign"
    }
    
    # Velocity constants
    NORMAL_VELOCITY = 0.3  # Normal driving speed
    SLOW_VELOCITY = 0.15   # Reduced speed for yield/slow signs
    STOP_VELOCITY = 0.0    # Full stop
    
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(TrafficSignNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        # Get vehicle name from environment
        self._vehicle_name = os.environ['VEHICLE_NAME']
        
        # Topic names
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._cmd_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        
        # Initialize CV bridge
        self._bridge = CvBridge()
        
        # Initialize AprilTag detector
        self._detector_options = apriltag.DetectorOptions(families="tag36h11")
        self._detector = apriltag.Detector(self._detector_options)
        
        # Current state variables
        self._current_velocity = self.NORMAL_VELOCITY
        self._current_omega = 0.0
        self._last_sign_detected = None
        self._sign_detection_time = None
        self._stop_duration = 7.0  # Stop for 7 seconds when stop sign detected
        self._slow_recovery_time = 3.0  # Time to recover from slow down signs
        
        # Publishers
        self._cmd_publisher = rospy.Publisher(self._cmd_topic, Twist2DStamped, queue_size=1)
        self._sign_publisher = rospy.Publisher(f"/{self._vehicle_name}/detected_signs", String, queue_size=1)
        self._debug_publisher = rospy.Publisher("traffic_sign_debug", CompressedImage, queue_size=1)
        
        # Subscribers
        self._camera_subscriber = rospy.Subscriber(self._camera_topic, CompressedImage, self.camera_callback)
        
        # Create debug window
        self._window_name = "Traffic Sign Detection"
        cv2.namedWindow(self._window_name, cv2.WINDOW_AUTOSIZE)
        
        rospy.loginfo(f"Traffic Sign Node initialized for vehicle: {self._vehicle_name}")

    def process_image(self, frame):
        """
        Process camera frame to detect AprilTags (traffic signs)
        
        Args:
            frame: OpenCV image frame
            
        Returns:
            dict: Contains processed frame and detection results
        """
        height, width = frame.shape[:2]
        
        # Process full frame instead of just right half for better coverage
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect AprilTags
        detections = self._detector.detect(gray)
        
        # Create visual feedback frame
        visual_frame = frame.copy()
        detected_signs = []
        
        for detection in detections:
            # Draw bounding box
            for i in range(4):
                pt1 = tuple(map(int, detection.corners[i]))
                pt2 = tuple(map(int, detection.corners[(i + 1) % 4]))
                cv2.line(visual_frame, pt1, pt2, (0, 255, 0), 2)
            
            # Draw center point
            center = tuple(map(int, detection.center))
            cv2.circle(visual_frame, center, 5, (0, 0, 255), -1)
            
            # Get sign type from tag ID
            sign_type = self.SIGN_MAPPING.get(detection.tag_id, "unknown")
            detected_signs.append({
                'tag_id': detection.tag_id,
                'sign_type': sign_type,
                'center': center,
                'distance': self.estimate_distance(detection)
            })
            
            # Add sign type label
            label = f"ID:{detection.tag_id} ({sign_type})"
            cv2.putText(visual_frame, label, (center[0]-50, center[1]-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        return {
            "frame": visual_frame,
            "detections": detections,
            "signs": detected_signs
        }
    
    def estimate_distance(self, detection):
        """
        Rough distance estimation based on tag size
        (This is a simplified approach - in practice you'd calibrate this)
        """
        # Calculate area of detected tag
        corners = detection.corners
        area = cv2.contourArea(corners.astype(np.int32))
        
        # Rough distance estimation (larger area = closer)
        if area > 5000:
            return "close"
        elif area > 2000:
            return "medium"
        else:
            return "far"
    
    def execute_sign_behavior(self, detected_signs):
        """
        Execute appropriate behavior based on detected signs
        
        Args:
            detected_signs: List of detected sign dictionaries
        """
        if not detected_signs:
            # No signs detected, maintain normal velocity
            if self._current_velocity != self.NORMAL_VELOCITY:
                rospy.loginfo(f"No signs detected - Resuming normal speed ({self.NORMAL_VELOCITY} m/s)")
                self._current_velocity = self.NORMAL_VELOCITY
            return
        
        # Process the closest/most relevant sign
        closest_sign = min(detected_signs, key=lambda s: 0 if s['distance'] == 'close' 
                          else 1 if s['distance'] == 'medium' else 2)
        
        sign_type = closest_sign['sign_type']
        current_time = rospy.Time.now()
        
        # Implement sign-specific behaviors (stop and slow down only)
        if sign_type == "stop_sign":
            if self._last_sign_detected != "stop_sign" or \
               (current_time - self._sign_detection_time).to_sec() > 5.0:
                # New stop sign detected or enough time passed
                rospy.loginfo(f"STOP SIGN detected - Stopping vehicle completely for {self._stop_duration} seconds")
                self._current_velocity = self.STOP_VELOCITY
                self._last_sign_detected = "stop_sign"
                self._sign_detection_time = current_time
            elif (current_time - self._sign_detection_time).to_sec() > self._stop_duration:
                # Stop duration completed, resume normal speed
                rospy.loginfo(f"Stop duration ({self._stop_duration}s) completed - Resuming normal speed")
                self._current_velocity = self.NORMAL_VELOCITY
                
        elif sign_type == "slow_down_sign":
            if self._last_sign_detected != "slow_down_sign" or \
               (current_time - self._sign_detection_time).to_sec() > 2.0:
                # New slow down sign detected or enough time passed
                rospy.loginfo(f"SLOW DOWN sign detected - Reducing speed to {self.SLOW_VELOCITY} m/s")
                self._current_velocity = self.SLOW_VELOCITY
                self._last_sign_detected = "slow_down_sign"
                self._sign_detection_time = current_time
            elif (current_time - self._sign_detection_time).to_sec() > self._slow_recovery_time:
                # Slow down duration completed, resume normal speed
                rospy.loginfo("Slow down period completed - Resuming normal speed")
                self._current_velocity = self.NORMAL_VELOCITY
            
        # Publish detected sign info
        sign_msg = String()
        sign_msg.data = f"{sign_type}:{closest_sign['distance']}"
        self._sign_publisher.publish(sign_msg)
    
    def camera_callback(self, msg):
        """
        Callback function for camera image messages
        
        Args:
            msg: CompressedImage message from camera
        """
        try:
            # Convert compressed image to OpenCV format
            frame = self._bridge.compressed_imgmsg_to_cv2(msg)
            
            # Process frame for traffic sign detection
            result = self.process_image(frame)
            
            # Execute behavior based on detected signs
            self.execute_sign_behavior(result['signs'])
            
            # Publish movement command
            self.publish_movement_command()
            
            # Display debug visualization
            cv2.imshow(self._window_name, result['frame'])
            cv2.waitKey(1)
            
            # Publish debug image
            debug_msg = self._bridge.cv2_to_compressed_imgmsg(result['frame'])
            self._debug_publisher.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"Error in camera callback: {str(e)}")
    
    def publish_movement_command(self):
        """
        Publish Twist2DStamped message with current velocity and omega
        """
        twist_msg = Twist2DStamped()
        twist_msg.v = self._current_velocity
        twist_msg.omega = self._current_omega
        twist_msg.header.stamp = rospy.Time.now()
        
        self._cmd_publisher.publish(twist_msg)
    
    def on_shutdown(self):
        """
        Cleanup function called when node is shutting down
        """
        rospy.loginfo("Traffic Sign Node shutting down")
        
        # Send stop command
        stop_msg = Twist2DStamped()
        stop_msg.v = 0.0
        stop_msg.omega = 0.0
        self._cmd_publisher.publish(stop_msg)
        
        # Close OpenCV windows
        cv2.destroyAllWindows()


if __name__ == '__main__':
    # Create the node
    node = TrafficSignNode(node_name='traffic_sign_node')
    
    # Set up shutdown hook
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep the node running
    rospy.spin()