#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#RESET_STATE
import sys
sys.path.append('/home/minh/agv_ws/src/agv_image_processing/yolov5-object-tracking')

from sort import Sort
import os
import math
import time
from pathlib import Path
from enum import Enum
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool
from agv_interfaces.msg import PersonTracking
from cv_bridge import CvBridge

# YOLOv5 imports
from utils.general import set_logging
from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams, letterbox
from utils.general import (LOGGER, Profile, check_file, check_img_size, 
                         check_imshow, check_requirements, colorstr, cv2,
                         increment_path, non_max_suppression, print_args,
                         scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import colors, save_one_box
from utils.torch_utils import select_device, time_sync

class RobotState(Enum):
    IDLE = "IDLE"
    DETECTING = "DETECTING"
    FOLLOWING = "FOLLOWING"
    STOPPING = "STOPPING"

class SafetyController:
    def __init__(self, start_delay, stop_delay):
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
        self.countdown = 0.0      
        self.reset_tracker_requested = False


    #roi: region of interest   
    def update_roi_bounds(self, img_width, img_height):
        self.roi_x1 = int(img_width * 0.35)
        self.roi_x2 = int(img_width * 0.7)
        self.roi_y1 = int(img_height * 0.2)
        self.roi_y2 = int(img_height * 0.7)
        
    def is_person_in_roi(self, person_center):
        if not self.roi_enabled:
            return True           
        cx, cy = person_center
        return (self.roi_x1 <= cx <= self.roi_x2 and 
                self.roi_y1 <= cy <= self.roi_y2)
    
    """Update state machine based on detection"""
    def update_state(self, person_detected, person_center=None, target_person_exists=False):
        current_time = time.time()
        person_in_roi = False
        
        # Only check ROI for IDLE and DETECTING states
        if self.state in [RobotState.IDLE, RobotState.DETECTING]:
            if person_detected and person_center:
                person_in_roi = self.is_person_in_roi(person_center)
                if person_in_roi:
                    self.last_detection_time = current_time
        else: 
            person_in_roi = target_person_exists  
            if target_person_exists:
                self.last_detection_time = current_time   
        
        # State machine logic
        if self.state == RobotState.IDLE:
            if person_in_roi:
                self.state = RobotState.DETECTING
                self.detection_start_time = current_time
                self.countdown = float(self.start_delay)
                
        elif self.state == RobotState.DETECTING:
            if person_in_roi:
                elapsed = current_time - self.detection_start_time
                self.countdown = max(0, float(self.start_delay) - elapsed)
                
                if elapsed >= self.start_delay:
                    self.state = RobotState.FOLLOWING
                    self.countdown = 0.0
            else:
                self.state = RobotState.IDLE
                self.detection_start_time = None
                self.countdown = 0.0
                
        elif self.state == RobotState.FOLLOWING:
            if not target_person_exists:  # Countdown when SORT cannot track target
                if self.stop_start_time is None:
                    self.stop_start_time = current_time
                    self.state = RobotState.STOPPING
                    self.countdown = float(self.stop_delay)
            else:
                # Target still exists, cancel stopping
                self.stop_start_time = None
                
        elif self.state == RobotState.STOPPING:
            if target_person_exists:  # Target back
                self.state = RobotState.FOLLOWING
                self.stop_start_time = None
                self.countdown = 0.0
            else:
                elapsed = current_time - self.stop_start_time
                self.countdown = max(0, float(self.stop_delay) - elapsed)
                
                if elapsed >= self.stop_delay:
                    self.state = RobotState.IDLE
                    self.stop_start_time = None
                    self.countdown = 0.0
                    
                    self.reset_tracker_requested = True # tracker reset when returned to IDLE
                    print("Returned to IDLE: reset tracker ")
        
        self.person_in_roi_status = person_in_roi  # Renamed variable
        return self.state
    

    """Check if should publish distance/angle data"""
    def should_publish_data(self):
        return self.state == RobotState.FOLLOWING
    
    def get_status_text(self): # Display to monitor     
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
        
        self.reset_tracker_requested = True
        print(" SafetyController: Requesting tracker reset ")
    
    def check_and_clear_reset_request(self):
        """Check if reset was requested and clear the flag"""
        if self.reset_tracker_requested:
            self.reset_tracker_requested = False
            return True
        return False

class CameraLidarFusion:
    def __init__(self, camera_fov=83.0, image_width=640):
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
        lidar_angle_deg = -camera_angle + self.calibration_offset
        lidar_angle_rad = math.radians(lidar_angle_deg)
        return lidar_angle_rad
    
    """Find person distance by matching angle LiDAR and camera"""
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
            point_angle = angle_min + (i * angle_increment)
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

class SORTTracker:
    """SORT-based person tracking to lock onto first person after countdown"""
    def __init__(self):
        self.tracker = Sort(max_age=30, min_hits=1, iou_threshold=0.3) 
        self.target_person_id = None  
        self.target_locked = False
        self.reset_requested = False
        
    def reset_tracking(self):
        """Reset tracking - will lock to next detected person after countdown"""
        self.target_person_id = None
        self.target_locked = False
        self.tracker = Sort(max_age=30, min_hits=1, iou_threshold=0.3) 
        self.reset_requested = True
    
    def update_tracking(self, detections, current_state=None):
        """
        Update SORT tracking and return target person info
        Args:
            detections: All person detections (in and out of ROI)
            current_state: Current robot state for lock timing
        Returns: dict with target person info or None
        """
        if not detections:
            # No detections - update tracker with empty array
            tracked_objects = self.tracker.update(np.empty((0, 6)))       
            # Check if target person still exists
            if self.target_person_id is not None:
                target_exists = any(int(obj[-1]) == self.target_person_id for obj in tracked_objects)
                if not target_exists:
                    # Target lost
                    return None
            return None
        
        # Convert detections to SORT format: [x1, y1, x2, y2, score, class]
        dets_for_sort = []
        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            dets_for_sort.append([x1, y1, x2, y2, conf, cls])
        
        dets_array = np.array(dets_for_sort)
        
        # Update SORT tracker with ALL detections (maintain ID consistency)
        tracked_objects = self.tracker.update(dets_array)
        
        if len(tracked_objects) == 0:
            return None
        # Lock when countdown is done 
        if not self.target_locked and current_state is not None:
            if current_state.value == "FOLLOWING":  # Chỉ lock khi countdown xong
                # Get all track IDs and find the minimum (first person)
                track_ids = [int(obj[-1]) for obj in tracked_objects]
                self.target_person_id = min(track_ids)
                self.target_locked = True
                print(f"LOCKED Target ID ")
        
        # Re-lock when lost target
        if self.target_locked and len(tracked_objects) > 0:
            current_track_ids = [int(obj[-1]) for obj in tracked_objects]
            
            if self.target_person_id not in current_track_ids:
                if len(current_track_ids) > 0:
                    old_id = self.target_person_id
                    self.target_person_id = min(current_track_ids)  # Re-lock to first available person
                else:
                    print(f" Target completely lost - no other persons available")
        
        # Find target person in tracked objects
        target_person = None
        for obj in tracked_objects:
            track_id = int(obj[-1])  # Last column is track ID
            if track_id == self.target_person_id:
                x1, y1, x2, y2 = obj[:4]
                
                # Create target person info in same format as original PersonSelector
                target_person = {
                    'detection': [x1, y1, x2, y2, 0.9, 0],  
                    'center': ((x1 + x2) / 2, (y1 + y2) / 2),
                    'area': (x2 - x1) * (y2 - y1),
                    'total_score': 1.0,  # Max score for target
                    'track_id': track_id
                }
                break
        
        return target_person
    
    def get_all_tracked_objects(self):
        """Return all currently tracked objects for visualization"""
        return getattr(self.tracker, 'trackers', [])

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('agv_person_detector')
        
        # Configuration
        weights = '/home/minh/yolov5-object-tracking/yolov5n.pt'
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
        
        # Publishers 
        self.image_pub = self.create_publisher(Image, '/yolov5/image', 1)
        self.tracking_pub = self.create_publisher(PersonTracking, '/person/tracking', 10)
        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.reset_sub = self.create_subscription(Bool, '/detector/reset', self.reset_callback, 10)
        self.emergency_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)
        
        self._init_model(weights)
        self._init_camera()
        
        self.sort_tracker = SORTTracker()
        self.fusion = CameraLidarFusion(camera_fov=83.0, image_width=640)
        
        # Safety controller
        self.safety_controller = SafetyController(start_delay=4.0, stop_delay=5.0)
        self.safety_controller.update_roi_bounds(640, 480)
        self.emergency_warning = False
        
        # Reduce frame rate
        self.frame_count = 0
        self.start_time = time.time()
        self.last_process_time = 0        
        
        # Main processing function
        self.timer = self.create_timer(0.05, self.process_frame)
        
    def lidar_callback(self, msg):
        """Callback cho LiDAR data"""
        self.fusion.update_lidar_data(msg)
    
    def reset_callback(self, msg):
        """Reset state machine when safety controller requests"""
        if msg.data:
            self.safety_controller.reset_state_machine()           
            self.get_logger().warn('State machine reset by external command - SORT tracker will be reset')
            # Publish immediate update
            self._publish_tracking_data(None, None)

    def emergency_callback(self, msg):
        """Callback cho emergency stop"""
        self.emergency_warning = msg.data
        
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
        self.cap = cv2.VideoCapture("/dev/video2", cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            rclpy.shutdown()
            return       
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 24)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    """Main processing function"""
    def process_frame(self):
        start_time = time.time()        
        if (start_time - self.last_process_time) < 0.03: 
            return           
        ret, img0 = self.cap.read()
        if not ret:
            return      
        detections = self._detect_persons(img0)
        target_person = self.sort_tracker.update_tracking(detections, None) 
        
        # check and handle reset request
        if self.safety_controller.check_and_clear_reset_request():
            self.sort_tracker.reset_tracking()
            self.get_logger().warn(' SORT Tracker reset')
          
            # Re-run tracking with current detections after reset
            target_person = self.sort_tracker.update_tracking(detections, None)
        
        # Declare detecting person variables
        person_in_roi_detected = False
        person_center_for_roi = None    
        if detections:
            for detection in detections:
                x1, y1, x2, y2, conf, cls = detection
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2            
                if self.safety_controller.is_person_in_roi((center_x, center_y)):
                    person_in_roi_detected = True
                    person_center_for_roi = (center_x, center_y)
                    break 
        
        # Update safety state
        current_state = self.safety_controller.update_state(
            person_detected=person_in_roi_detected,
            person_center=person_center_for_roi,
            target_person_exists=(target_person is not None) 
        )      
        # Re-run SORT for locking target 
        target_person = self.sort_tracker.update_tracking(detections, current_state)
        
        # Calculate distance using LiDAR
        person_distance = None
        camera_angle = None
        lidar_angle = None
        if target_person:  # Target person (có thể ở ngoài ROI)
            center_x, center_y = target_person['center']
            camera_angle = self.fusion.pixel_to_camera_angle(center_x)            
            # Only get distance if we should publish (FOLLOWING state)
            if self.safety_controller.state == RobotState.FOLLOWING:
                person_distance, lidar_angle = self.fusion.find_person_distance(camera_angle)
            
            # Log info với state
            if person_distance is not None:
                if self.frame_count % 30 == 0:
                    self.get_logger().info(
                        f"State: {current_state.value} | "
                        f"Target ID: {target_person.get('track_id', 'N/A')} | "
                        f"Person - angle: {camera_angle:.1f}°, "
                        f"distance: {person_distance:.2f}m"
                    )             
        # UPDATED: Publish tracking data using PersonTracking message
        self._publish_tracking_data(camera_angle, person_distance)       
        
        result_img = self._draw_results(img0, target_person, camera_angle, person_distance)       
        self._publish_image(result_img)        
        self._update_stats(start_time)
        
    def _publish_tracking_data(self, camera_angle, person_distance):
        """Publish all tracking data in single PersonTracking message"""
        msg = PersonTracking()
        msg.timestamp = self.get_clock().now().to_msg()
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
        # Preprocess
        img = letterbox(img0, self.imgsz, stride=self.stride, auto=False)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1) # BGR to RGB
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
                det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], img0.shape).round()
                for detection in det:
                    if int(detection[5]) == 0:  # person class
                        person_detections.append(detection.cpu().numpy())       
        return person_detections        
    
    def _draw_results(self, img0, target_person, camera_angle, person_distance):
        """Draw detection results on image with ROI and status"""
        result_img = img0.copy()
        
        # Draw ROI rectangle (different style when FOLLOWING)
        if self.safety_controller.roi_enabled:
            # Only show active ROI for IDLE and DETECTING states
            if self.safety_controller.state in [RobotState.IDLE, RobotState.DETECTING]:
                roi_color = (0, 255, 0) if self.safety_controller.person_in_roi_status else (0, 0, 255)
                roi_thickness = 2
            else:  # FOLLOWING or STOPPING - ROI not active
                roi_color = (128, 128, 128)  
                roi_thickness = 1  
                
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
            RobotState.IDLE: (255, 0, 0),          # Blue
            RobotState.DETECTING: (0, 255, 255),   # Yellow
            RobotState.FOLLOWING: (0, 255, 0),     # Green
            RobotState.STOPPING: (0, 0, 255)       # Red
        }.get(self.safety_controller.state, (255, 255, 255))
        
        cv2.putText(result_img, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
        
        if self.sort_tracker.target_locked:
            target_id_text = f"TARGET LOCKED"
            cv2.putText(result_img, target_id_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            if self.safety_controller.state == RobotState.DETECTING:
                waiting_text = f"WAITING TO LOCK TARGET..."
                cv2.putText(result_img, waiting_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if self.safety_controller.countdown > 0:
            countdown_text = f"{self.safety_controller.countdown:.1f}s"
            cv2.putText(result_img, countdown_text, (500, 100), 
                       cv2.FONT_HERSHEY_SIMPLEX, 2.0, state_color, 3)
        
        if target_person:
            detection = target_person['detection']
            x1, y1, x2, y2, conf, cls = detection                  
            # Convert to int coordinates
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)           
            # Target person always use GREEN (locked target)
            bbox_color = (0, 255, 0)  # Green for locked target
                      # Draw bounding box
            cv2.rectangle(result_img, (x1, y1), (x2, y2), bbox_color, self.line_thickness)         
            # Create label with Track ID
            track_id = target_person.get('track_id', 'N/A')
            if person_distance is not None:
                label = f'TARGET ID:{track_id} | {camera_angle:.1f} | {person_distance:.2f}m'
            else:
                label = f'TARGET ID:{track_id} | {camera_angle:.1f} | No LiDAR'
            
            # Draw label background
            (label_width, label_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            
            # Background rectangle for label
            cv2.rectangle(result_img, 
                         (x1, y1 - label_height - baseline - 5),
                         (x1 + label_width + 5, y1), 
                         bbox_color, -1)  # Filled rectangle
            
            # Draw label text
            cv2.putText(result_img, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Draw center point
            cx, cy = map(int, target_person['center'])
            cv2.circle(result_img, (cx, cy), 4, bbox_color, -1)
            
            # Draw angle line (always when target locked)
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

        current_time = time.time()
        blink = int(current_time * 3) % 2  # Nhấp nháy 1.5Hz
        
        if self.emergency_warning and blink:
            # Chỉ hiện khi blink = 1 (nhấp nháy)
            cv2.rectangle(result_img, (0, 0), (640, 80), (0, 0, 255), -1)
            cv2.putText(result_img, "EMERGENCY STOP!!!", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)

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
