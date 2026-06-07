#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from agv_interfaces.msg import PersonTracking
from std_msgs.msg import Bool, String, Float32, Int8
import math

class UltrasonicSafetyNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_safety_node')
        
        # Safety parameters
        self.emergency_stop_distance = 0.25  # 25cm - emergency stop
        self.slow_distance = 0.40         
        
        # Current sensor readings
        self.front_distance = float('inf')
        self.rear_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # Safety states
        self.emergency_stop_signal = False
        self.reduce_speed = False
        
        # Person tracking state
        self.following_state = "UNKNOWN"
        self.last_scan_time = None
        
        # Publishers
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.speed_limit_pub = self.create_publisher(Float32, '/speed_limit_factor', 10)
        self.led_status_pub = self.create_publisher(Int8, '/led_states', 10)
        
        # Subscribers for ultrasonic sensors
        self.front_sub = self.create_subscription(
            Range, '/ultrasonic/front', self.front_callback, 10)
        self.rear_sub = self.create_subscription(
            Range, '/ultrasonic/rear', self.rear_callback, 10)
        self.left_sub = self.create_subscription(
            Range, '/ultrasonic/left', self.left_callback, 10)
        self.right_sub = self.create_subscription(
            Range, '/ultrasonic/right', self.right_callback, 10)
        
        # Person tracking subscriber
        self.person_tracking_sub = self.create_subscription(
            PersonTracking, '/person/tracking', self.person_tracking_callback, 10)
        
        # Timer for safety logic and publishing
        self.safety_timer = self.create_timer(0.05, self.safety_check)  # 20Hz
        
        self.get_logger().info('Ultrasonic Safety Node started')
        self.get_logger().info(f'Emergency stop distance: {self.emergency_stop_distance}m')
        self.get_logger().info(f'Slow speed distance: {self.slow_distance}m')
    
    def person_tracking_callback(self, msg):
        """Update person tracking data"""
        self.following_state = msg.state_name
        
    def front_callback(self, msg):
        """Update front sensor reading"""
        self.front_distance = msg.range
        self.last_scan_time = self.get_clock().now()
        
    def rear_callback(self, msg):
        """Update rear sensor reading"""
        self.rear_distance = msg.range
        
    def left_callback(self, msg):
        """Update left sensor reading"""
        self.left_distance = msg.range
        
    def right_callback(self, msg):
        """Update right sensor reading"""
        self.right_distance = msg.range
        
    def safety_check(self):
        """Main safety logic - runs at 20Hz"""
        # Get minimum distance based on state
        if self.following_state == "FOLLOWING":
            # Khi following, chỉ xét cảm biến phía trước
            min_distance = self.front_distance
        else:
            # Các trạng thái khác xét tất cả cảm biến
            min_distance = min(self.front_distance, self.rear_distance, 
                             self.left_distance, self.right_distance)
        
        # Check for emergency stop condition
        if min_distance < self.emergency_stop_distance:
            if not self.emergency_stop_signal:
                self.get_logger().error(f'EMERGENCY STOP! Obstacle at {min_distance:.2f}m')
            self.emergency_stop_signal = True
            self.reduce_speed = True
        elif min_distance < self.slow_distance:
            if self.emergency_stop_signal:
                self.get_logger().info('Emergency condition cleared')
            self.emergency_stop_signal = False
            self.reduce_speed = True
        else:
            if self.emergency_stop_signal or self.reduce_speed:
                self.get_logger().info('All clear - normal speed')
            self.emergency_stop_signal = False
            self.reduce_speed = False
            
        # Truyền min_distance vào publish_safety_status
        self.publish_safety_status(min_distance)
        
    def publish_safety_status(self, min_distance):
        """Publish safety information"""
        stop_msg = Bool()
        stop_msg.data = self.emergency_stop_signal
        self.emergency_stop_pub.publish(stop_msg)
        
        # Speed limit factor (Float32)
        speed_msg = Float32()
        if self.emergency_stop_signal:
            speed_msg.data = 0.0
        elif self.reduce_speed:
            speed_msg.data = 0.5
        else:
            speed_msg.data = 1.0
        self.speed_limit_pub.publish(speed_msg)
        
        # LED Status based on following state and emergency
        led_msg = Int8()
        if self.emergency_stop_signal:
            led_msg.data = 4 
        elif self.following_state == "FOLLOWING":
            led_msg.data = 2  # Following state
        elif self.following_state == "STOPPING":
            led_msg.data = 3  # Stopping state
        elif self.following_state == "IDLE":
            led_msg.data = 1  # Idle state
        self.led_status_pub.publish(led_msg)

        # Log status periodically
        if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # Every second
            if self.emergency_stop_signal:
                self.get_logger().info(f'Safety Status - Emergency Stop Active')
    
    def destroy_node(self):
        """Clean shutdown"""
        # Send all clear signals
        stop_msg = Bool()
        stop_msg.data = False
        self.emergency_stop_pub.publish(stop_msg)
        
        speed_msg = Float32()
        speed_msg.data = 1.0
        self.speed_limit_pub.publish(speed_msg)
        
        self.get_logger().info('Ultrasonic Safety Node shutting down')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        ultrasonic_safety = UltrasonicSafetyNode()
        rclpy.spin(ultrasonic_safety)
    except KeyboardInterrupt:
        pass
    finally:
        if 'ultrasonic_safety' in locals():
            ultrasonic_safety.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()