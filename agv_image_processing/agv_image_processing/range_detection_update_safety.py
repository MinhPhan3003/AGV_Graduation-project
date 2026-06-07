#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import torch
import sys
sys.path.append('/home/minh/agv_ws/src/agv_image_processing/yolov5')
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool  
# Import custom message instead of std_msgs
from agv_interfaces.msg import PersonTracking
from cv_bridge import CvBridge
import cv2
import time
from pathlib import Path
import math
from enum import Enum

from models.common import DetectMultiBackend
from utils.general import (check_img_size, non_max_suppression, scale_boxes)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox

class RobotState(Enum):
    IDLE = "IDLE"
    DETECTING = "DETECTING"
    FOLLOWING = "FOLLOWING"
    STOPPING = "STOPPING"

class SafetyController:
    def __init__(self, start_delay=5.0, stop_delay=3.0):
        self.roi_enabled = True
        self.start_delay = start_delay
        self.stop_delay = stop_delay
        self.state = RobotState.IDLE       
        # Timers
        self.detection_start_time = None
        self.stop_start_time = None
        self.last_detection_time = None      
        # Tracking info
        self.person_in_roi_status = False  
        self.countdown = 0        
    def update_roi_bounds(self, img_width, img_height):
        self.roi_x1 = int(img_width * 0.3)
        self.roi_x2 = int(img_width * 0.7)
        self.roi_y1 = int(img_height * 0.2)
        self.roi_y2 = int(img_height * 0.9)
        
    def is_person_in_roi(self, person_center):
        if not self.roi_enabled:
            return True           
        cx, cy = person_center
        return (self.roi_x1 <= cx <= self.roi_x2 and 
                self.roi_y1 <= cy <= self.roi_y2)
    
    def update_state(self, person_detected, person_center=None):
        """Update state machine based on detection"""
        current_time = time.time()
        person_in_roi = False
        
        # Only check ROI for IDLE and DETECTING states
        if self.state in [RobotState.IDLE, RobotState.DETECTING]:
            if person_detected and person_center:
                person_in_roi = self.is_person_in_roi(person_center)
                if person_in_roi:
                    self.last_detection_time = current_time
        else: 
            person_in_roi = person_detected  # Just need person detected
            if person_detected:
                self.last_detection_time = current_time   
        # State machine logic
        if self.state == RobotState.IDLE:
            if person_in_roi:
                self.state = RobotState.DETECTING
                self.detection_start_time = current_time
                self.countdown = self.start_delay
                
        elif self.state == RobotState.DETECTING:
            if person_in_roi:
                elapsed = current_time - self.detection_start_time
                self.countdown = max(0, self.start_delay - elapsed)
                
                if elapsed >= self.start_delay:
                    self.state = RobotState.FOLLOWING
                    self.countdown = 0
            else:
                self.state = RobotState.IDLE
                self.detection_start_time = None
                self.countdown = 0
                
        elif self.state == RobotState.FOLLOWING:
            if not person_detected:  # Changed from person_in_roi
                if self.stop_start_time is None:
                    self.stop_start_time = current_time
                    self.state = RobotState.STOPPING
                    self.countdown = self.stop_delay
            else:
                # Person still detected, cancel stopping
                self.stop_start_time = None
                
        elif self.state == RobotState.STOPPING:
            if person_detected:  # Changed from person_in_roi
                # Person came back, resume following
                self.state = RobotState.FOLLOWING
                self.stop_start_time = None
                self.countdown = 0
            else:
                elapsed = current_time - self.stop_start_time
                self.countdown = max(0, self.stop_delay - elapsed)
                
                if elapsed >= self.stop_delay:
                    self.state = RobotState.IDLE
                    self.stop_start_time = None
                    self.countdown = 0
        
        self.person_in_roi_status = person_in_roi  # Renamed variable
        return self.state
    
    def should_publish_data(self):
        """Check if should publish distance/angle data"""
        return self.state == RobotState.FOLLOWING
    
    def get_status_text(self):
        """Get status text for display"""
        if self.state == RobotState.IDLE:
            return "Waiting for person in detection zone"
        elif self.state == RobotState.DETECTING:
            return f"Person detected! Starting in {self.countdown:.1f}s"
        elif self.state == RobotState.FOLLOWING:
            return "Following person"
        elif self.state == RobotState.STOPPING:
            return f"Person lost! Stopping in {self.countdown:.1f}s"
        return ""
    
    def reset_state_machine(self):
        """Public method to reset state machine"""
        self.state = RobotState.IDLE
        self.detection_start_time = None
        self.stop_start_time = None
        self.countdown = 0.0
        self.person_in_roi_status = False

class CameraLidarFusion:
    """Camera-LiDAR fusion for distance measurement"""
    def __init__(self, camera_fov=45.0, image_width=640):
        self.camera_fov = camera_fov 
        self.image_width = image_width
        self.image_center_x = image_width // 2        
        # LiDAR parameters 
        self.lidar_angle_min = -1.6580628156661987
        self.lidar_angle_max = 1.6580628156661987  
        self.lidar_angle_increment = 0.00581718236207962        
        self.calibration_offset = 0.025  # between lidar and camera angle       
        # LiDAR data storage
        self.latest_lidar_data = None
        self.lidar_timestamp = None
        
    def pixel_to_camera_angle(self, pixel_x):
        pixel_offset = pixel_x - self.image_center_x       
        angle_per_pixel = self.camera_fov / self.image_width
        camera_angle = pixel_offset * angle_per_pixel
        return camera_angle
    
    def camera_angle_to_lidar_angle(self, camera_angle):
        """Convert camera angle to LiDAR angle"""
        lidar_angle_deg = -camera_angle + self.calibration_offset
        lidar_angle_rad = math.radians(lidar_angle_deg)
        return lidar_angle_rad
    
    def find_person_distance(self, camera_angle, tolerance_deg=3.0):
        if self.latest_lidar_data is None:
            return None, None
        
        # convert camera angle to lidar angle
        target_lidar_angle = self.camera_angle_to_lidar_angle(camera_angle)
        tolerance_rad = math.radians(tolerance_deg)
        
        # find suitable index in LiDAR data
        angle_min = self.lidar_angle_min
        angle_increment = self.lidar_angle_increment
        valid_points = []
       
        for i, distance in enumerate(self.latest_lidar_data.ranges):
            if distance <= 0 or distance >= self.latest_lidar_data.range_max:
                continue                
            # calculate at index i:
            point_angle = angle_min + i * angle_increment
            angle_diff = abs(point_angle - target_lidar_angle)
                             
            if angle_diff <= tolerance_rad:
                valid_points.append({
                    'index': i,
                    'angle': point_angle,
                    'distance': distance,
                    'angle_diff': angle_diff
                })
        if not valid_points:
            return None, None
        # choose point near person_target
        best_point = min(valid_points, key=lambda x: x['angle_diff'])
        
        return best_point['distance'], math.degrees(best_point['angle'])    
    
    def update_lidar_data(self, lidar_msg):
        """update latest data from Lidar """
        self.latest_lidar_data = lidar_msg
        self.lidar_timestamp = time.time()

class PersonSelector:
    """Smart person selection based on multiple criteria"""
    def __init__(self, img_width, img_height):
        self.img_width = img_width
        self.img_height = img_height
        self.img_center_x = img_width // 2
        self.img_center_y = img_height // 2
        
        # Weights for selection criteria (simplified - removed aspect_ratio and stability)
        self.weights = {
            'confidence': 0.5,    # Increased from 0.3
            'size': 0.3,         # Increased from 0.25
            'centrality': 0.4    # Increased from 0.25
        }
        
    def _calculate_scores(self, detections):
        """Calculate scores for each detection"""
        scores = []
        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            
            # Confidence score
            conf_score = float(conf)
            
            # Size score - prefer moderate sizes
            width = x2 - x1
            height = y2 - y1
            area = width * height
            area_ratio = area / (self.img_width * self.img_height)
            
            if 0.1 <= area_ratio <= 0.4:
                size_score = 1.0
            elif area_ratio < 0.1:
                size_score = area_ratio / 0.1
            else:
                size_score = max(0, 1.0 - (area_ratio - 0.4) / 0.3)
            
            # Centrality score
            bbox_center_x = (x1 + x2) / 2
            bbox_center_y = (y1 + y2) / 2
            
            distance_to_center = np.sqrt(
                ((bbox_center_x - self.img_center_x) / self.img_width) ** 2 +
                ((bbox_center_y - self.img_center_y) / self.img_height) ** 2
            )
            centrality_score = max(0, 1.0 - distance_to_center * 2)
            
            # Calculate weighted total score (simplified)
            total_score = (
                conf_score * self.weights['confidence'] +
                size_score * self.weights['size'] +
                centrality_score * self.weights['centrality']
            )           
            scores.append({
                'detection': detection,
                'total_score': total_score,
                'center': (bbox_center_x, bbox_center_y),
                'area': area
            })  
        return scores
    
    def select_best_person(self, detections):
        """Select the best person based on criteria"""
        if not detections:
            return None       
        scores = self._calculate_scores(detections)
        scores.sort(key=lambda x: x['total_score'], reverse=True)
        best_person = scores[0]
        return best_person

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('agv_person_detector')
        
        # Configuration
        weights = '/home/minh/agv_ws/src/agv_image_processing/yolov5/yolov5n.pt'
        self.imgsz = (320, 256)
        self.conf_thres = 0.5
        self.iou_thres = 0.45
        self.max_det = 1000
        self.classes = [0]  # Person class only
        self.agnostic_nms = False
        self.line_thickness = 2
        self.bridge = CvBridge()
        # Device setup
        self.device = select_device('')
        self.half = self.device.type != 'cpu'
        
        # Publishers - REPLACED WITH SINGLE PUBLISHER
        self.image_pub = self.create_publisher(Image, '/yolov5/image', 1)
        self.tracking_pub = self.create_publisher(PersonTracking, '/person/tracking', 10)
        
        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.reset_sub = self.create_subscription(Bool, '/detector/reset', self.reset_callback, 10)
        
        self._init_model(weights)
        self._init_camera()
        
        # Person selector
        self.person_selector = PersonSelector(640, 480)
        # Camera-LiDAR fusion
        self.fusion = CameraLidarFusion(camera_fov=45.0, image_width=640)
        
        # Safety controller
        self.safety_controller = SafetyController(start_delay=5.0, stop_delay=3.0)
        self.safety_controller.update_roi_bounds(640, 480)
        
        # Performance monitoring
        self.frame_count = 0
        self.start_time = time.time()
        self.last_process_time = 0        
        
        # Timer for processing
        self.timer = self.create_timer(0.033, self.process_frame)
        
    def lidar_callback(self, msg):
        """Callback cho LiDAR data"""
        self.fusion.update_lidar_data(msg)
    
    def reset_callback(self, msg):
        """Reset state machine when safety controller requests"""
        if msg.data:
            # Reset to IDLE state using the public method
            self.safety_controller.reset_state_machine()
            
            # Log the reset
            self.get_logger().warn('State machine reset by safety controller - returning to IDLE')
            
            # Publish immediate update
            self._publish_tracking_data(None, None)
        
    def _init_model(self, weights):
        """Initialize YOLO model"""
        self.model = DetectMultiBackend(weights, device=self.device, dnn=False, fp16=self.half)
        self.stride, self.names, pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(self.imgsz, s=self.stride)
        
        if self.half and hasattr(self.model.model, 'half'):
            self.model.model.half()
        self.model.warmup(imgsz=(1 if pt else 1, 3, *self.imgsz))
        
    def _init_camera(self):
        """Initialize camera"""
        self.cap = cv2.VideoCapture("/dev/video3", cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            rclpy.shutdown()
            return       
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 24)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
    def process_frame(self):
        """Main processing function"""
        start_time = time.time()        
        # Skip if processing too slow
        if (start_time - self.last_process_time) < 0.03:
            return
            
        # Read frame
        ret, img0 = self.cap.read()
        if not ret:
            return
        
        # Process image
        detections = self._detect_persons(img0)
        best_person = self.person_selector.select_best_person(detections) if detections else None
        
        # Update safety state
        person_center = None
        if best_person:
            person_center = best_person['center']
        
        current_state = self.safety_controller.update_state(
            person_detected=(best_person is not None),
            person_center=person_center
        )        
        # Calculate distance using LiDAR
        person_distance = None
        camera_angle = None
        lidar_angle = None
        
        if best_person:  # Remove ROI check for FOLLOWING state
            center_x, center_y = best_person['center']
            camera_angle = self.fusion.pixel_to_camera_angle(center_x)
            
            # Only get distance if we should publish (FOLLOWING state)
            if self.safety_controller.state == RobotState.FOLLOWING:
                person_distance, lidar_angle = self.fusion.find_person_distance(camera_angle)
            
            # Log info với state
            if person_distance is not None:
                self.get_logger().info(
                    f"State: {current_state.value} | "
                    f"Person - angle: {camera_angle:.1f}°, "
                    f"distance: {person_distance:.2f}m"
                )       
        
        # UPDATED: Publish tracking data using PersonTracking message
        self._publish_tracking_data(camera_angle, person_distance)       
        
        # Draw results với ROI và status
        result_img = self._draw_results(img0, best_person, camera_angle, person_distance)       
        self._publish_image(result_img)        
        self._update_stats(start_time)
        
    def _publish_tracking_data(self, camera_angle, person_distance):
        """Publish all tracking data in single PersonTracking message"""
        msg = PersonTracking()
        
        # Set timestamp
        msg.timestamp = self.get_clock().now().to_msg()
        
        # Set state information
        msg.following_state = self.safety_controller.should_publish_data()
        msg.state_name = self.safety_controller.state.value
        msg.countdown = self.safety_controller.countdown
        
        # Set tracking data based on state
        if self.safety_controller.should_publish_data():
            # Valid data when following
            msg.angle = float(camera_angle) if camera_angle is not None else 0.0
            msg.distance = float(person_distance) if person_distance is not None else -1.0
        else:
            # Invalid data when not following
            msg.angle = 0.0
            msg.distance = -1.0
            
        # Publish the message
        self.tracking_pub.publish(msg)
        
        # Log published data periodically
        if self.frame_count % 30 == 0:  # Every second at 30fps
            self.get_logger().info(
                f"Published: state={msg.state_name}, "
                f"following={msg.following_state}, "
                f"distance={msg.distance:.2f}, "
                f"angle={msg.angle:.1f}, "
                f"countdown={msg.countdown:.1f}"
            )
        
    def _detect_persons(self, img0):
        """Detect persons in image"""
        # Preprocess
        img = letterbox(img0, self.imgsz, stride=self.stride, auto=False)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)       
        # Convert to tensor
        img_tensor = torch.from_numpy(img).to(self.device)
        img_tensor = img_tensor.half() if self.half else img_tensor.float()
        img_tensor /= 255.0
        
        if len(img_tensor.shape) == 3:
            img_tensor = img_tensor[None]
        
        # Inference
        with torch.no_grad():
            pred = self.model(img_tensor)[0]
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, 
                                     self.classes, self.agnostic_nms, max_det=self.max_det)
        
        # Extract person detections
        person_detections = []
        for det in pred:
            if len(det):
                det[:, :4] = scale_boxes(img_tensor.shape[2:], det[:, :4], img0.shape).round()
                for detection in det:
                    if int(detection[5]) == 0:  # person class
                        person_detections.append(detection.cpu().numpy())
        
        return person_detections        
        
    def _draw_results(self, img0, best_person, camera_angle, person_distance):
        """Draw detection results on image with ROI and status"""
        result_img = img0.copy()
        
        # Draw ROI rectangle (different style when FOLLOWING)
        if self.safety_controller.roi_enabled:
            # Only show active ROI for IDLE and DETECTING states
            if self.safety_controller.state in [RobotState.IDLE, RobotState.DETECTING]:
                roi_color = (0, 255, 0) if self.safety_controller.person_in_roi_status else (0, 0, 255)
                roi_thickness = 2
            else:  # FOLLOWING or STOPPING - ROI not active
                roi_color = (128, 128, 128)  # Gray
                roi_thickness = 1  # Thinner line
                
            cv2.rectangle(result_img, 
                         (self.safety_controller.roi_x1, self.safety_controller.roi_y1),
                         (self.safety_controller.roi_x2, self.safety_controller.roi_y2),
                         roi_color, roi_thickness)
            
            # Draw ROI label
            roi_text = "Detection Zone" if self.safety_controller.state in [RobotState.IDLE, RobotState.DETECTING] else "Detection Zone (inactive)"
            cv2.putText(result_img, roi_text, 
                       (self.safety_controller.roi_x1, self.safety_controller.roi_y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, roi_color, 2)
        
        # Draw status text
        status_text = self.safety_controller.get_status_text()
        state_color = {
            RobotState.IDLE: (128, 128, 128),      # Gray
            RobotState.DETECTING: (0, 255, 255),   # Yellow
            RobotState.FOLLOWING: (0, 255, 0),     # Green
            RobotState.STOPPING: (0, 0, 255)       # Red
        }.get(self.safety_controller.state, (255, 255, 255))
        
        cv2.putText(result_img, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
        
        # Draw countdown if applicable
        if self.safety_controller.countdown > 0:
            countdown_text = f"{self.safety_controller.countdown:.1f}s"
            cv2.putText(result_img, countdown_text, (280, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 2.0, state_color, 3)
        
        if best_person:
            detection = best_person['detection']
            x1, y1, x2, y2, conf, cls = detection        
            
            # Different color based on state and ROI
            if self.safety_controller.state in [RobotState.IDLE, RobotState.DETECTING]:
                bbox_color = colors(0, True) if self.safety_controller.person_in_roi_status else (128, 128, 128)
            else:  # FOLLOWING or STOPPING - always use active color
                bbox_color = colors(0, True)
            
            # Draw bounding box
            annotator = Annotator(result_img, line_width=self.line_thickness, example=str(self.names))            
            
            # Create label with angle and distance
            if person_distance is not None:
                label = f'Person {conf:.2f} | {camera_angle:.1f}° | {person_distance:.2f}m'
            else:
                label = f'Person {conf:.2f} | {camera_angle:.1f}° | No LiDAR'
                
            annotator.box_label([x1, y1, x2, y2], label, color=bbox_color)
            result_img = annotator.result()            
            
            # Draw center point
            cx, cy = map(int, best_person['center'])
            if self.safety_controller.state in [RobotState.IDLE, RobotState.DETECTING]:
                point_color = (0, 255, 0) if self.safety_controller.person_in_roi_status else (128, 128, 128)
            else:  # FOLLOWING or STOPPING
                point_color = (0, 255, 0)  # Always green
            cv2.circle(result_img, (cx, cy), 4, point_color, -1)
            
            # Draw angle line (always when person detected)
            if camera_angle is not None:
                center_img_x = result_img.shape[1] // 2
                line_length = 100
                angle_rad = math.radians(camera_angle)
                end_x = int(center_img_x + line_length * math.sin(angle_rad))
                end_y = int(cy)
                cv2.line(result_img, (center_img_x, cy), (end_x, end_y), (255, 0, 0), 2)
               
        # Draw state indicator in corner
        state_indicator_text = f"[{self.safety_controller.state.value}]"
        cv2.putText(result_img, state_indicator_text, (520, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, state_color, 2)       
        return result_img
        
    def _publish_image(self, img):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish: {e}")
            
    def _update_stats(self, start_time):
        """Update performance statistics"""
        self.frame_count += 1
        process_time = time.time() - start_time
        self.last_process_time = start_time    
        if self.frame_count % 60 == 0:
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed
            self.get_logger().info(f"FPS: {fps:.1f}, Process: {process_time*1000:.1f}ms")
    
    def destroy_node(self):
        """Clean shutdown - publish final invalid message"""
        final_msg = PersonTracking()
        final_msg.timestamp = self.get_clock().now().to_msg()
        final_msg.distance = -1.0
        final_msg.angle = 0.0
        final_msg.following_state = False
        final_msg.state_name = "IDLE"
        final_msg.countdown = 0.0
        self.tracking_pub.publish(final_msg)
        
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()