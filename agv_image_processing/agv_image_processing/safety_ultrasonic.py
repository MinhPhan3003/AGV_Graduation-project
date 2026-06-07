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
        self.emergency_stop_distance = 0.30 
        self.emergency_clear_distance = 0.35 
        
        # Current sensor readings
        self.front_distance = float('inf')
        self.rear_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # Safety states
        self.emergency_stop_signal = False
        self.following_state = "UNKNOWN"
        self.last_scan_time = None
        self.emergency_trigger_sensor = ""
        # ===== DOUBLE-CHECK LOGIC TO PREVENT FALSE ALARMS =====
        self.emergency_check_count = 0           
        self.last_emergency_distance = float('inf')  
        
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
        
        # Person tracking subscribe
        self.person_tracking_sub = self.create_subscription(
            PersonTracking, '/person/tracking', self.person_tracking_callback, 10)
        
        self.safety_timer = self.create_timer(0.1, self.safety_check)  
        self.get_logger().info(f'Ultrasonic safety has started')
    
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
        
    def find_closest_sensor(self):
        """Find which sensor has the closest reading and return sensor info"""
        sensors = {
            'FRONT': self.front_distance,
            'REAR': self.rear_distance,
            'LEFT': self.left_distance,
            'RIGHT': self.right_distance
        }   
        min_sensor = min(sensors, key=sensors.get)
        min_distance = sensors[min_sensor]
        return min_distance, min_sensor, sensors
        
    def safety_check(self):      
        min_distance, trigger_sensor, all_sensors = self.find_closest_sensor()
        if self.emergency_stop_signal:
            if min_distance > self.emergency_clear_distance:
                self.emergency_stop_signal = False
                self.emergency_check_count = 0
                self.emergency_trigger_sensor = ""
        else:
            if min_distance < self.emergency_stop_distance:
                self.emergency_check_count += 1
                self.last_emergency_distance = min_distance
                
                if self.emergency_check_count >= 2:
                    self.emergency_stop_signal = True
                    self.emergency_trigger_sensor = trigger_sensor
                    self.get_logger().error(f'EMERGENCY STOP TRIGGERED!')
            else:
                self.emergency_check_count = 0
        
        self.publish_safety_status(min_distance, trigger_sensor, all_sensors)
        
    def publish_safety_status(self, min_distance, trigger_sensor, all_sensors):
        stop_msg = Bool()
        stop_msg.data = self.emergency_stop_signal
        self.emergency_stop_pub.publish(stop_msg)
        
        # ===== SPEED LIMIT FACTOR - SIMPLIFIED (NO SLOW SPEED) =====
        speed_msg = Float32()
        if self.emergency_stop_signal:
            speed_msg.data = 0.0  # FULL STOP
        else:
            speed_msg.data = 1.0  # FULL SPEED
        self.speed_limit_pub.publish(speed_msg)
        
        # ===== LED STATUS WITH EMERGENCY PRIORITY =====
        led_msg = Int8()
        if self.emergency_stop_signal:
            led_msg.data = 4  # Emergency
        elif self.following_state == "FOLLOWING":
            led_msg.data = 2  # Following
        elif self.following_state == "STOPPING":
            led_msg.data = 3  # Stopping
        elif self.following_state == "IDLE":
            led_msg.data = 1  # Idle
        else:
            led_msg.data = 0  # Unknown
        self.led_status_pub.publish(led_msg)
        if self.emergency_stop_signal and self.get_clock().now().nanoseconds % 200000000 < 50000000:
            self.get_logger().error(f' EMERGENCY ACTIVE! ')
                                  
    
    def destroy_node(self):
        self.get_logger().info('Safety Node shutting down')
        stop_msg = Bool()
        stop_msg.data = False
        self.emergency_stop_pub.publish(stop_msg)    
        speed_msg = Float32()
        speed_msg.data = 1.0
        self.speed_limit_pub.publish(speed_msg)       
        led_msg = Int8()
        led_msg.data = 0  # Turn off LEDs
        self.led_status_pub.publish(led_msg)
    
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    ultrasonic_safety = UltrasonicSafetyNode()
    rclpy.spin(ultrasonic_safety)
    ultrasonic_safety.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()