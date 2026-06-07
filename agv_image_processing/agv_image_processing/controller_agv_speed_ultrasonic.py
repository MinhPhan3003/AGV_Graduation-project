#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from agv_interfaces.msg import PersonTracking
from std_msgs.msg import Bool, Float32
from rclpy.time import Time
from builtin_interfaces.msg import Time
import time
import math

class SafetyController(Node):
    def __init__(self):
        super().__init__('agv_safety_controller')
        # Safety parameters
        self.min_safe_distance = 0.55
        self.max_safe_distance = 4.5  
        self.target_distance = 1.0    
        self.distance_tolerance = 0.1        
        self.max_safe_angle = 40.0
        self.angle_tolerance = 5.0
        
        # Velocity limits
        self.max_linear_vel = 0.38
        self.max_angular_vel = 0.43
        self.reduced_linear_vel = 0.15
        
        # PI Controller parameters
        self.linear_kp = 0.38         
        self.linear_ki = 0.02          
        self.angular_kp = 0.02         
        self.angular_ki = 0.002        
        self.last_control_time = None
        
        # Integral terms
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.max_integral = 0.5
        
        # Subscribers
        self.tracking_sub = self.create_subscription(
            PersonTracking, '/person/tracking', self.tracking_callback, 10)       
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10)
        self.speed_limit_sub = self.create_subscription(
            Float32, '/speed_limit_factor', self.speed_limit_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'agv_controller/cmd_vel', 10)
        self.reset_pub = self.create_publisher(Bool, '/detector/reset', 10)      
        
        # State variables from PersonTracking
        self.current_distance = -1.0
        self.current_angle = 0.0
        self.detector_following = False
        self.detector_state = "IDLE"
        self.detector_countdown = 0.0
        self.last_msg_time = None
        
        # Safety state
        self.safety_following = False
        self.safety_stop_reason = ""
        self.stop_timer = 0.0
        self.stop_delay = 5.0
        
        # Too close safety timer
        self.too_close_timer = 0.0
        self.too_close_delay = 5.0
        self.has_sent_reset = False
        
        # Emergency stop from ultrasonic
        self.emergency_stop_signal = False
        self.emergency_stop_reason = ""
        
        # Speed limit: 1.0 = full speed, 0.0 = stop
        self.ultrasonic_speed_limit = 1.0 
        
        # Control timer - 20Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)       
        self.get_logger().info('Safety Controller started with ultrasonic integration')
        self.get_logger().info(f'Safe distance: {self.min_safe_distance}m - {self.max_safe_distance}m')
        
    def tracking_callback(self, msg):
        """Update all tracking data from single message"""
        self.current_distance = msg.distance
        self.current_angle = msg.angle
        self.detector_following = msg.following_state
        self.detector_state = msg.state_name
        self.detector_countdown = msg.countdown
        current_time = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.timestamp)
        self.last_msg_time = current_time
        
        # Monitor state changes
        if hasattr(self, 'prev_state') and self.prev_state != msg.state_name:
            self.get_logger().info(f'Detector state: {self.prev_state} → {msg.state_name}')
        self.prev_state = msg.state_name

    def emergency_stop_callback(self, msg):
        """Handle emergency stop from ultrasonic sensors"""
        self.emergency_stop_signal = msg.data
        if msg.data:
            self.emergency_stop_reason = "obstacle detected"
            if self.get_clock().now().nanoseconds % 2000000000 < 50000000:
                self.get_logger().error('EMERGENCY STOP!!!')
        else:
            if self.emergency_stop_reason:
                self.get_logger().info('OBSTACLE CLEARED')
            self.emergency_stop_reason = ""

    def speed_limit_callback(self, msg):
        """Handle speed limit factor from ultrasonic sensors"""
        self.ultrasonic_speed_limit = msg.data
        # Log significant speed changes
        if hasattr(self, 'prev_speed_limit'):
            if abs(self.prev_speed_limit - msg.data) > 0.1:
                if msg.data == 0.0:
                    self.get_logger().warn('Ultrasonic enforcing STOP')
                elif msg.data == 0.5:
                    self.get_logger().warn('Ultrasonic enforcing SLOW speed (50%)')
                elif msg.data == 1.0:
                    self.get_logger().info('Ultrasonic allowing FULL speed')
        self.prev_speed_limit = msg.data

    def check_safety_conditions(self):
        """Enhanced safety check with ultrasonic integration"""
        # PRIORITY 1: EMERGENCY STOP from ultrasonic
        if self.emergency_stop_signal:
            self.safety_stop_reason = self.emergency_stop_reason
            return True  # FIXED: Emergency stop should return False
            
        # PRIORITY 2: DATA AVAILABLE?
        if self.last_msg_time is None:
            self.safety_stop_reason = "No data received"
            return False
            
        # PRIORITY 3: CHECK DATA AGE
        current_time = self.get_clock().now()
        age = (current_time - self.last_msg_time).nanoseconds / 1e9
        if age > 0.5:
            self.safety_stop_reason = f"Stale data ({age:.2f}s old)"
            return False
        
        # PRIORITY 4: DETECTOR FOLLOWING  
        if not self.detector_following:
            self.safety_stop_reason = f"Detector: {self.detector_state}"
            if self.detector_countdown > 0:
                self.safety_stop_reason += f" ({self.detector_countdown:.1f}s)"
            return False
            
        # PRIORITY 5: DISTANCE VALIDITY
        if self.current_distance < 0:
            self.safety_stop_reason = "Invalid distance"
            return False
            
        # PRIORITY 6: SAFETY DISTANCE AND ANGLE
        if self.current_distance < self.min_safe_distance:
            if self.too_close_timer == 0.0:
                self.too_close_timer = time.time()
                self.get_logger().warn(f"Person too close: {self.current_distance:.2f}m")            
            elapsed = time.time() - self.too_close_timer
            if elapsed < self.too_close_delay:
                remaining = self.too_close_delay - elapsed
                self.safety_stop_reason = f"Too close (waiting {remaining:.1f}s)"
                return True  
            else:
                self.safety_stop_reason = f"Person too close timeout"
                return False
        else:
            if self.too_close_timer > 0:
                self.get_logger().info("Person moved to safe distance")
            self.too_close_timer = 0.0
            self.has_sent_reset = False
            
        if self.current_distance > self.max_safe_distance:
            self.safety_stop_reason = f"Person too far: {self.current_distance:.2f}m"
            return False           

        if abs(self.current_angle) > self.max_safe_angle:
            self.safety_stop_reason = f"Angle too large: {abs(self.current_angle):.1f}°"
            return False    
            
        # All checks passed
        self.safety_stop_reason = ""
        return True
        
    def calculate_velocities(self, dt):
        """Calculate velocities with ultrasonic speed limiting"""
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = "base_footprint"
    
        # Angular control
        angle_error = self.current_angle   
        if abs(angle_error) > self.angle_tolerance:
            self.angular_integral += angle_error * dt
            self.angular_integral = max(-self.max_integral/self.angular_ki, 
                                      min(self.max_integral/self.angular_ki, self.angular_integral))
            
            cmd_vel.twist.angular.z = -(self.angular_kp * angle_error + self.angular_ki * self.angular_integral)
            cmd_vel.twist.angular.z = max(-self.max_angular_vel, 
                                        min(self.max_angular_vel, cmd_vel.twist.angular.z))
        else:
            cmd_vel.twist.angular.z = 0.0
            self.angular_integral = 0.0

        # Linear control
        distance_error = self.current_distance - self.target_distance     
        
        # Stop if person too close and waiting
        if self.too_close_timer > 0:
            cmd_vel.twist.linear.x = 0.0
            self.linear_integral = 0.0
            return cmd_vel
        
        if abs(distance_error) > self.distance_tolerance:
            self.linear_integral += distance_error * dt
            self.linear_integral = max(-self.max_integral/self.linear_ki, 
                                     min(self.max_integral/self.linear_ki, self.linear_integral))
            
            cmd_vel.twist.linear.x = self.linear_kp * distance_error + self.linear_ki * self.linear_integral
            if cmd_vel.twist.linear.x < 0:
                cmd_vel.twist.linear.x = 0.0
                if self.linear_integral < 0:
                    self.linear_integral = 0.0           
            
            # Progressive speed based on person distance
            if self.current_distance < 0.7:
                max_vel = self.reduced_linear_vel * 0.5
            #elif self.current_distance < 1.4:
                #max_vel = self.reduced_linear_vel
            else:
                max_vel = self.max_linear_vel
                
            cmd_vel.twist.linear.x = min(max_vel, cmd_vel.twist.linear.x)
        else:
            cmd_vel.twist.linear.x = 0.0
            self.linear_integral = 0.0

        # APPLY ULTRASONIC SPEED LIMIT
        cmd_vel.twist.linear.x *= self.ultrasonic_speed_limit
        cmd_vel.twist.angular.z *= self.ultrasonic_speed_limit   
        return cmd_vel
        
    def control_loop(self):
        """Main control loop with enhanced safety"""
        current_control_time = self.get_clock().now()
        if self.last_control_time is not None:
            dt = (current_control_time - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = current_control_time

        # Check safety conditions
        safe_to_follow = self.check_safety_conditions()
        
        if self.emergency_stop_signal:
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.header.frame_id = "base_footprint"
            cmd_vel.twist.linear.x = 0.0
            cmd_vel.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            return
                
        if safe_to_follow and not self.safety_following:
            self.safety_following = True
            self.stop_timer = 0.0
            self.get_logger().info('Starting to follow')
            
        elif not safe_to_follow and self.safety_following:
            if self.stop_timer == 0.0:
                self.stop_timer = time.time()
                self.get_logger().warn(f'Safety failed: {self.safety_stop_reason}')
                
            # Immediate stop conditions
            if (self.emergency_stop_signal or 
                (self.current_distance > 0 and 
                 self.current_distance < self.min_safe_distance and 
                 self.too_close_timer > 0 and 
                 time.time() - self.too_close_timer >= self.too_close_delay)):
                
                self.safety_following = False
                self.stop_timer = 0.0
                
                if self.emergency_stop_signal:
                    self.get_logger().error('IMMEDIATE STOP - OBSTACLE!')
                else:
                    self.get_logger().error('IMMEDIATE STOP - Person too close!')
                    if not self.has_sent_reset:
                        reset_msg = Bool()
                        reset_msg.data = True
                        self.reset_pub.publish(reset_msg)
                        self.has_sent_reset = True
                        self.get_logger().warn('Reset detector - need new detection')
                        
            elif time.time() - self.stop_timer > self.stop_delay:
                self.safety_following = False
                self.stop_timer = 0.0
                self.get_logger().info('Normal stop after delay')
                
        elif safe_to_follow and self.stop_timer > 0:
            self.stop_timer = 0.0
            self.get_logger().info('Safety restored - continuing')
            
        # Reset integrals when not following
        if not self.safety_following:
            self.linear_integral = 0.0
            self.angular_integral = 0.0
            
        # Generate and publish commands
        if self.safety_following:
            cmd_vel = self.calculate_velocities(dt)
            
            # Periodic logging
            if self.get_clock().now().nanoseconds % 2000000000 < 50000000:
                log_msg = (f'Following - Dist: {self.current_distance:.2f}m, '
                          f'Angle: {self.current_angle:.1f}°, '
                          f'Vel: [{cmd_vel.twist.linear.x:.2f}, {cmd_vel.twist.angular.z:.2f}]')
                
                if self.ultrasonic_speed_limit < 1.0:
                    log_msg += f' | Speed limit: {self.ultrasonic_speed_limit*100:.0f}%'
                    
                if self.detector_state != "FOLLOWING":
                    log_msg += f' | Detector: {self.detector_state}'
                    
                if self.too_close_timer > 0:
                    elapsed = time.time() - self.too_close_timer
                    log_msg += f' | Too close: {elapsed:.1f}s'
                    
                self.get_logger().info(log_msg)
        else:
            # Stop command
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.header.frame_id = "base_footprint"
                        
            if self.stop_timer > 0:
                remaining = self.stop_delay - (time.time() - self.stop_timer)
                if remaining > 0 and self.get_clock().now().nanoseconds % 1000000000 < 50000000:
                    self.get_logger().warn(f'Stopping in {remaining:.1f}s - {self.safety_stop_reason}')
                    
        self.cmd_vel_pub.publish(cmd_vel)
        
    def destroy_node(self):
        """Clean shutdown"""
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.get_clock().now().to_msg()
        stop_cmd.header.frame_id = "base_footprint"
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('Safety Controller shutting down')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        safety_controller = SafetyController()
        rclpy.spin(safety_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'safety_controller' in locals():
            safety_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
