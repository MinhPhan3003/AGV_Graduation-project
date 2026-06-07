#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped  # Changed from Twist
from agv_interfaces.msg import PersonTracking
from std_msgs.msg import Bool
from rclpy.time import Time
from builtin_interfaces.msg import Time
import time
import math

class SafetyController(Node):
    def __init__(self):
        super().__init__('agv_safety_controller')
        
        # Safety parameters
        self.min_safe_distance = 0.7  # meters - increased for safety
        self.max_safe_distance = 3.5  
        self.target_distance = 1.0    
        self.distance_tolerance = 0.1 
        self.max_safe_angle = 30.0   # degrees
        self.angle_tolerance = 5.0    # degrees (dead zone: -5 to +5)
        
        # Velocity limits
        self.max_linear_vel = 0.5     
        self.max_angular_vel = 1.0    
        self.reduced_linear_vel = 0.2 
        
        # Control gains - UPDATED TO PI CONTROLLER
        self.linear_kp = 0.43          
        self.linear_ki = 0.05          
        self.angular_kp = 0.02         
        self.angular_ki = 0.005        
        
        # Integral terms
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.max_integral = 0.5        # Anti-windup limit
        
        # Subcriber and Publisher
        self.tracking_sub = self.create_subscription(
            PersonTracking, '/person/tracking', self.tracking_callback, 10)
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10)
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
        self.stop_delay = 3.0  # seconds before full stop
        
        # Too close safety timer
        self.too_close_timer = 0.0
        self.too_close_delay = 3.0  # seconds to wait when person too close
        self.has_sent_reset = False  # Track if reset was sent
        
        # External emergency stop
        self.external_emergency_stop = False
        self.emergency_stop_reason = ""
        
        # Control timer - 20Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Safety Controller started')
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
        
        # Monitor: Observe behaviours
        if hasattr(self, 'prev_state') and self.prev_state != msg.state_name:
            self.get_logger().info(f'Detector state changed: {self.prev_state} → {msg.state_name}')
        self.prev_state = msg.state_name        

    def emergency_stop_callback(self, msg):
        self.external_emergency_stop = msg.data
        if msg.data:
            self.emergency_stop_reason = "Obstacle detected "
            self.get_logger().error('EMERGENCY STOP !!!')
        else:
            if self.emergency_stop_reason:
                self.get_logger().info('PATH CLEARED')
            self.emergency_stop_reason = ""
        
    def check_safety_conditions(self):
        """Check if all safety conditions are met"""
        # PRIORITY 1: EMERGENCY STOP!!!
        if self.external_emergency_stop:
            self.safety_stop_reason = self.emergency_stop_reason
            return False
            
        # PRIORITY 2: DATA AVAILABLE?
        if self.last_msg_time is None:
            self.safety_stop_reason = "No data received"
            return False
            
        # PRIORITY 3: CHECK DATA DURATION (0.5s)
        current_time = self.get_clock().now()
        age = (current_time - self.last_msg_time).nanoseconds / 1e9
        if age > 0.5:
            self.safety_stop_reason = f"Stale data ({age:.2f}s old)"
            return False
        
        # PRIORITY 4: DETECTOR IN FOLLOWING MODE 
        if not self.detector_following:
            self.safety_stop_reason = f"Detector state: {self.detector_state}"
            if self.detector_countdown > 0:
                self.safety_stop_reason += f" (countdown: {self.detector_countdown:.1f}s)"
            return False
            
        # PRIORITY 5: CHECK DISTANCE VALIDITY
        if self.current_distance < 0:
            self.safety_stop_reason = "Invalid distance"
            return False
            
        # PRIORITY 6: SAFETY DISTANCE AND ANGLE
        if self.current_distance < self.min_safe_distance:
            # Start or continue too close timer
            if self.too_close_timer == 0.0:
                self.too_close_timer = time.time()
                self.get_logger().warn(f"Person too close: {self.current_distance:.2f}m")           
            elapsed = time.time() - self.too_close_timer
            if elapsed < self.too_close_delay:
                remaining = self.too_close_delay - elapsed
                self.safety_stop_reason = f"Too close (waiting {remaining:.1f}s)"
                return True 
            else:
                self.safety_stop_reason = f"TIME OUT FOR WAITING"
                return False
        else:
            # Reset timer
            if self.too_close_timer > 0:
                self.get_logger().info("Person moved to safe distance")
            self.too_close_timer = 0.0
            self.has_sent_reset = False  # Reset the flag when safe
            
        if self.current_distance > self.max_safe_distance:
            self.safety_stop_reason = f"Too far: {self.current_distance:.2f}m > {self.max_safe_distance}m"
            return False
            
        # Safety angle check
        if abs(self.current_angle) > self.max_safe_angle:
            self.safety_stop_reason = f"Angle too large: {abs(self.current_angle):.1f}° > {self.max_safe_angle}°"
            return False
            
        # All checks passed
        self.safety_stop_reason = ""
        return True
        
    def calculate_velocities(self):
        """Calculate linear and angular velocities with PI controller - returns TwistStamped"""
        cmd_vel = TwistStamped()
        
        # Set header with current timestamp
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = "base_link"
        
        # Time step for integral calculation (50ms)
        dt = 0.05
        
        # Angular velocity PI controller - turn towards person
        angle_error = self.current_angle  # degrees
        
        # Apply dead zone for angle
        if abs(angle_error) > self.angle_tolerance:
            # Update integral term
            self.angular_integral += angle_error * dt
            # Anti-windup: limit integral
            self.angular_integral = max(-self.max_integral/self.angular_ki, 
                                      min(self.max_integral/self.angular_ki, self.angular_integral))
            
            # PI controller for angular velocity
            cmd_vel.twist.angular.z = -(self.angular_kp * angle_error + self.angular_ki * self.angular_integral)
            
            # Limit angular velocity
            cmd_vel.twist.angular.z = max(-self.max_angular_vel, 
                                        min(self.max_angular_vel, cmd_vel.twist.angular.z))
        else:
            cmd_vel.twist.angular.z = 0.0
            # Reset integral when in dead zone
            self.angular_integral = 0.0
            
        # Linear velocity PI controller - maintain target distance
        distance_error = self.current_distance - self.target_distance
        
        # Check if person is too close and we're waiting
        if self.too_close_timer > 0:
            # Person too close - just stop, don't move backward
            cmd_vel.twist.linear.x = 0.0
            # Reset integral
            self.linear_integral = 0.0
            return cmd_vel
        if abs(distance_error) > self.distance_tolerance:
            # Update integral term
            self.linear_integral += distance_error * dt
            # Anti-windup: limit integral
            self.linear_integral = max(-self.max_integral/self.linear_ki, 
                                     min(self.max_integral/self.linear_ki, self.linear_integral))
            
            # PI controller for linear velocity
            cmd_vel.twist.linear.x = self.linear_kp * distance_error + self.linear_ki * self.linear_integral
            
            # IMPORTANT: Only allow forward movement or stop
            if cmd_vel.twist.linear.x < 0:
                cmd_vel.twist.linear.x = 0.0  # Stop instead of moving backward
                # Reset integral if trying to go backward
                if self.linear_integral < 0:
                    self.linear_integral = 0.0
            
            # Progressive speed reduction based on distance
            if self.current_distance < 1.2:
                max_vel = self.reduced_linear_vel * 0.5  # Very slow when very close
            elif self.current_distance < 1.4:
                max_vel = self.reduced_linear_vel  # Slow when close
            else:
                max_vel = self.max_linear_vel  # Normal speed
                
            # Limit linear velocity (only positive/forward)
            cmd_vel.twist.linear.x = min(max_vel, cmd_vel.twist.linear.x)
        else:
            cmd_vel.twist.linear.x = 0.0
            # Reset integral when in dead zone
            self.linear_integral = 0.0
            
        return cmd_vel
        
    def control_loop(self):
        """Main control loop - runs at 20Hz"""
        # Check safety conditions
        safe_to_follow = self.check_safety_conditions()
        
        # State transition logic
        if safe_to_follow and not self.safety_following:
            # Start following
            self.safety_following = True
            self.stop_timer = 0.0
            self.get_logger().info('Safety conditions met - Starting to follow')
            
        elif not safe_to_follow and self.safety_following:
            # Lost safety - start stop timer
            if self.stop_timer == 0.0:
                self.stop_timer = time.time()
                self.get_logger().warn(f'Safety condition failed: {self.safety_stop_reason}')
                
            # Check if we should stop immediately
            if (self.current_distance > 0 and 
                self.current_distance < self.min_safe_distance and 
                self.too_close_timer > 0 and 
                time.time() - self.too_close_timer >= self.too_close_delay):
                # Person stayed too close for too long
                self.safety_following = False
                self.stop_timer = 0.0
                self.get_logger().error(f'Emergency stop - Person too close for {self.too_close_delay}s!')
                
                # Send reset signal to detector if not already sent
                if not self.has_sent_reset:
                    reset_msg = Bool()
                    reset_msg.data = True
                    self.reset_pub.publish(reset_msg)
                    self.has_sent_reset = True
                    self.get_logger().warn('Sent reset signal to detector - requiring new 5s detection')
                    
            elif time.time() - self.stop_timer > self.stop_delay:
                # Normal stop after delay
                self.safety_following = False
                self.stop_timer = 0.0
                self.get_logger().info('Stopping after safety delay')
                
        elif safe_to_follow and self.stop_timer > 0:
            # Safety restored during stop countdown
            self.stop_timer = 0.0
            self.get_logger().info('Safety conditions restored - Continuing to follow')
            
        # Reset integrals when not following
        if not self.safety_following:
            self.linear_integral = 0.0
            self.angular_integral = 0.0
            
        # Publish velocity commands
        if self.safety_following:
            cmd_vel = self.calculate_velocities()
            
            # Log current state periodically - UPDATE FOR TWISTSTAMPED
            if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # Log every second
                log_msg = (f'Following - Distance: {self.current_distance:.2f}m, '
                          f'Angle: {self.current_angle:.1f}°, '
                          f'Vel: [{cmd_vel.twist.linear.x:.2f}, {cmd_vel.twist.angular.z:.2f}]')
                
                # Add detector state info
                if self.detector_state != "FOLLOWING":
                    log_msg += f' | Detector: {self.detector_state}'
                    if self.detector_countdown > 0:
                        log_msg += f' ({self.detector_countdown:.1f}s)'
                
                # Add safety warnings
                if self.too_close_timer > 0:
                    elapsed = time.time() - self.too_close_timer
                    log_msg += f' | WARNING: Too close for {elapsed:.1f}s'
                
                # Add integral terms to log
                if abs(self.linear_integral) > 0.01 or abs(self.angular_integral) > 0.01:
                    log_msg += f' | I_lin: {self.linear_integral:.3f}, I_ang: {self.angular_integral:.3f}'
                    
                self.get_logger().info(log_msg)
        else:
            # Stop robot - CREATE TWISTSTAMPED WITH ZEROS
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.header.frame_id = "base_footprint"
            # twist values are already zeros by default
            
            if self.stop_timer > 0:
                remaining = self.stop_delay - (time.time() - self.stop_timer)
                if remaining > 0 and self.get_clock().now().nanoseconds % 1000000000 < 50000000:
                    self.get_logger().warn(f'Stopping in {remaining:.1f}s - {self.safety_stop_reason}')
                    
        self.cmd_vel_pub.publish(cmd_vel)
        
    def destroy_node(self):
        """Clean shutdown"""
        # Stop robot with TwistStamped
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.get_clock().now().to_msg()
        stop_cmd.header.frame_id = "base_footprint"
        # twist values are zeros by default
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('Safety Controller shutting down - Robot stopped')
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