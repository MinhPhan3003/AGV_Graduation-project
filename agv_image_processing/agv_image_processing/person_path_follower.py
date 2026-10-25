#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Person following by tracing the person's own path (breadcrumb trail).

The old controller steers straight at the person. When the person turns a
corner the robot cuts the corner and drives into the wall, and as soon as the
person leaves the camera view it stops.

This node stores every person position in the odom frame and follows that
trail with pure pursuit. When the person turns, the robot keeps driving to the
point where the person turned and only then turns.

LiDAR is used to:
  - refine the person position (snap the camera estimate to a leg/body cluster)
  - keep tracking the person with LiDAR alone for a short time when the camera
    loses them (e.g. they walked out of the camera field of view)
  - slow down / stop for obstacles in the driving corridor
When the person is lost completely, the robot drives to the last seen position
and turns in the direction the person was walking so the camera can find them.

Speed control is a cascade of two PID loops:
  - outer loop (position): distance left along the trail -> linear velocity
    setpoint, heading error to the pure pursuit point -> angular velocity
    setpoint. The integral term removes the lag behind a walking person.
  - inner loop (velocity, optional): setpoint vs measured velocity from wheel
    odometry (and the IMU gyro for the yaw rate) -> correction added to the
    command, so the robot really reaches the speed the outer loop asks for.
"""
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32
from agv_interfaces.msg import PersonTracking

DEFAULT_PARAMS = {
    # Following
    'follow_distance': 1.0,          # desired distance along the trail [m]
    'min_safe_distance': 0.55,       # straight-line distance to stop [m]
    'too_close_delay': 5.0,          # too close this long -> reset detector [s]
    'lost_approach_distance': 0.3,   # stop this far from last seen position [m]
    'search_extra_angle': 45.0,      # when lost, look this much further into the turn [deg]
    # Trail
    'breadcrumb_spacing': 0.15,
    'waypoint_reached_dist': 0.25,
    'lookahead_distance': 0.5,       # smaller = follows corners more tightly
    'max_trail_points': 300,
    # Velocity
    'max_linear_vel': 0.38,
    'max_angular_vel': 0.43,
    # Outer loop: distance along the trail -> linear velocity setpoint
    'linear_kp': 0.5,
    'linear_ki': 0.2,
    'linear_kd': 0.0,
    'linear_i_limit': 0.3,           # max integral contribution [m/s]
    'linear_i_leak': 0.9,           # integral decay per step inside follow distance
    # Outer loop: heading error -> angular velocity setpoint
    'angular_kp': 1.2,
    'angular_ki': 0.0,
    'angular_kd': 0.1,
    'angular_i_limit': 0.2,          # [rad/s]
    # Inner loop: velocity setpoint vs measured velocity -> correction
    'use_velocity_loop': False,
    'use_imu_yaw_rate': False,       # measured yaw rate from /imu/out instead of odom
    'vel_linear_kp': 0.3,
    'vel_linear_ki': 1.0,
    'vel_linear_limit': 0.1,         # max correction [m/s]
    'vel_angular_kp': 0.3,
    'vel_angular_ki': 1.0,
    'vel_angular_limit': 0.2,        # max correction [rad/s]
    'rotate_in_place_angle': 60.0,   # heading error above this: turn first [deg]
    'heading_tolerance': 5.0,        # [deg]
    # Person position estimate
    'lidar_x_offset': 0.366,         # LiDAR position in front of base_footprint [m]
    'lidar_angle_offset_deg': 0.025,  # same calibration as range_detection
    'max_person_range': 4.5,
    'position_filter_alpha': 0.6,
    'max_jump': 1.0,                 # larger jumps are treated as outliers [m]
    'max_person_speed': 1.5,         # widens max_jump after a gap in updates [m/s]
    'jump_confirm_count': 5,         # consecutive outliers -> new target
    'person_timeout': 0.5,           # no update this long -> person lost [s]
    # LiDAR person tracking
    'cluster_gap': 0.12,
    'max_cluster_width': 0.6,        # longer segments (walls) are ignored
    'min_cluster_points': 3,
    'camera_snap_radius': 0.4,
    'lidar_track_radius': 0.35,
    'lidar_only_timeout': 3.0,       # LiDAR-only tracking after camera loss [s]
    # Obstacles, measured in the LiDAR frame (LiDAR is at the robot front)
    'robot_half_width': 0.25,
    'obstacle_stop_dist': 0.25,
    'obstacle_slow_dist': 0.8,
}


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def to_robot_frame(pose, point):
    rx, ry, yaw = pose
    dx, dy = point[0] - rx, point[1] - ry
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def to_odom_frame(pose, point):
    rx, ry, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return rx + c * point[0] - s * point[1], ry + s * point[0] + c * point[1]


class PID:
    """PID with integral clamp, conditional integration and filtered D term."""

    def __init__(self, kp, ki, kd, i_limit, d_filter=0.7):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.d_filter = d_filter
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.d_state = 0.0

    def update(self, error, dt, out_min=-float('inf'), out_max=float('inf'), integrate=True):
        if dt <= 0.0 or dt > 0.5:
            # First call or a long pause: no reliable dt for I and D
            self.prev_error = error
            return max(out_min, min(out_max, self.kp * error + self.ki * self.integral))

        if self.kd > 0.0 and self.prev_error is not None:
            raw_d = (error - self.prev_error) / dt
            self.d_state = self.d_filter * self.d_state + (1.0 - self.d_filter) * raw_d
        self.prev_error = error

        old_integral = self.integral
        if integrate and self.ki > 0.0:
            self.integral += error * dt
            limit = self.i_limit / self.ki
            self.integral = max(-limit, min(limit, self.integral))

        out = self.kp * error + self.ki * self.integral + self.kd * self.d_state
        # Anti-windup: do not keep integrating while the output is saturated
        if (out > out_max and error > 0.0) or (out < out_min and error < 0.0):
            self.integral = old_integral
            out = self.kp * error + self.ki * self.integral + self.kd * self.d_state
        return max(out_min, min(out_max, out))


def process_scan(ranges, angle_min, angle_increment, range_min, range_max, p):
    """Return (clusters, front_clearance) in the LiDAR frame.

    clusters: list of (x, y, n_points) for short segments (legs, bodies).
    front_clearance: closest point in the driving corridor ahead [m].
    """
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(r)) * angle_increment
    valid = np.isfinite(r) & (r > max(range_min, 0.05)) & (r < range_max)
    x = r * np.cos(angles)
    y = r * np.sin(angles)

    corridor = valid & (x > 0.0) & (np.abs(y) < p['robot_half_width'])
    front_clearance = float(np.min(x[corridor])) if np.any(corridor) else float('inf')

    idx = np.nonzero(valid & (r < p['max_person_range'] + 1.0))[0]
    clusters = []
    if len(idx) == 0:
        return clusters, front_clearance
    px, py = x[idx], y[idx]
    gaps = np.hypot(np.diff(px), np.diff(py))
    gap_limit = np.maximum(p['cluster_gap'], 3.0 * r[idx][1:] * abs(angle_increment))
    breaks = (gaps > gap_limit) | (np.diff(idx) > 2)
    for seg in np.split(np.arange(len(idx)), np.nonzero(breaks)[0] + 1):
        if len(seg) < p['min_cluster_points']:
            continue
        width = math.hypot(px[seg[-1]] - px[seg[0]], py[seg[-1]] - py[seg[0]])
        if width > p['max_cluster_width']:
            continue
        clusters.append((float(np.mean(px[seg])), float(np.mean(py[seg])), len(seg)))
    return clusters, front_clearance


class TrailFollower:
    """ROS-independent trail logic, positions in the odom frame."""

    def __init__(self, params):
        self.p = params
        self.trail = deque(maxlen=int(params['max_trail_points']))
        # Last trail points, kept even after the robot passed them (for heading)
        self.recent = deque(maxlen=8)
        self.linear_pid = PID(params['linear_kp'], params['linear_ki'],
                              params['linear_kd'], params['linear_i_limit'])
        self.heading_pid = PID(params['angular_kp'], params['angular_ki'],
                               params['angular_kd'], params['angular_i_limit'])
        self.last_command_time = None
        self.reset()

    def reset(self):
        self.trail.clear()
        self.recent.clear()
        self.person = None
        self.person_heading = None
        self.person_turn_dir = 0         # +1 turning left, -1 turning right, 0 straight
        self.last_person_update = None
        self.last_camera_update = None
        self.jump_count = 0
        self.linear_pid.reset()
        self.heading_pid.reset()

    def lidar_to_odom(self, pose, point):
        return to_odom_frame(pose, (point[0] + self.p['lidar_x_offset'], point[1]))

    def camera_observation(self, pose, distance, camera_angle_deg):
        """Person position (odom frame) from the detector's distance and angle."""
        # Same convention as CameraLidarFusion: LiDAR angle = -camera angle
        bearing = math.radians(-camera_angle_deg + self.p['lidar_angle_offset_deg'])
        point = (distance * math.cos(bearing), distance * math.sin(bearing))
        return self.lidar_to_odom(pose, point)

    @staticmethod
    def match_clusters(clusters_odom, center, radius):
        """Weighted centroid of all clusters near center (joins two legs)."""
        sx = sy = n = 0.0
        for cx, cy, cnt in clusters_odom:
            if math.hypot(cx - center[0], cy - center[1]) <= radius:
                sx += cx * cnt
                sy += cy * cnt
                n += cnt
        if n == 0:
            return None
        return sx / n, sy / n

    def update_person(self, obs, now, from_camera):
        if self.person is not None:
            jump = math.hypot(obs[0] - self.person[0], obs[1] - self.person[1])
            allowed = self.p['max_jump'] + self.p['max_person_speed'] * (now - self.last_person_update)
            if jump > allowed:
                self.jump_count += 1
                if self.jump_count < self.p['jump_confirm_count']:
                    return False
                # Consistently somewhere else: the detector switched target
                self.reset()
            else:
                self.jump_count = 0
        if self.person is None:
            self.person = obs
        else:
            a = self.p['position_filter_alpha']
            self.person = (a * obs[0] + (1 - a) * self.person[0],
                           a * obs[1] + (1 - a) * self.person[1])

        if not self.trail:
            self.trail.append(self.person)
            self.recent.append(self.person)
        else:
            lx, ly = self.trail[-1]
            if math.hypot(self.person[0] - lx, self.person[1] - ly) >= self.p['breadcrumb_spacing']:
                self.trail.append(self.person)
                self.recent.append(self.person)
                ref = self.recent[-4] if len(self.recent) >= 4 else self.recent[0]
                if math.hypot(self.person[0] - ref[0], self.person[1] - ref[1]) > 0.1:
                    self.person_heading = math.atan2(self.person[1] - ref[1],
                                                     self.person[0] - ref[0])
                if len(self.recent) == self.recent.maxlen:
                    # Compare the older and the newer part of the trail
                    a, b = self.recent[0], self.recent[-4]
                    turn = wrap_angle(math.atan2(self.person[1] - b[1], self.person[0] - b[0]) -
                                      math.atan2(b[1] - a[1], b[0] - a[0]))
                    self.person_turn_dir = 0 if abs(turn) < math.radians(20) else (1 if turn > 0 else -1)

        self.last_person_update = now
        if from_camera:
            self.last_camera_update = now
        return True

    def person_visible(self, now):
        return (self.last_person_update is not None and
                now - self.last_person_update < self.p['person_timeout'])

    def lidar_tracking_allowed(self, now):
        """Camera lost the person recently: track with LiDAR only."""
        if self.person is None or self.last_camera_update is None:
            return False
        since_camera = now - self.last_camera_update
        return 0.3 < since_camera < self.p['lidar_only_timeout']

    def prune_trail(self, pose):
        rx, ry, _ = pose
        while len(self.trail) > 1:
            c0, c1 = self.trail[0], self.trail[1]
            reached = math.hypot(c0[0] - rx, c0[1] - ry) < self.p['waypoint_reached_dist']
            # Robot already past c0 along the segment c0 -> c1
            passed = (rx - c0[0]) * (c1[0] - c0[0]) + (ry - c0[1]) * (c1[1] - c0[1]) > 0
            if not (reached or passed):
                break
            self.trail.popleft()
        if len(self.trail) == 1 and self.person is not None:
            c0 = self.trail[0]
            if (math.hypot(c0[0] - rx, c0[1] - ry) < self.p['waypoint_reached_dist'] and
                    math.hypot(c0[0] - self.person[0], c0[1] - self.person[1]) > 1e-6):
                self.trail.popleft()

    def path_points(self, pose):
        pts = [(pose[0], pose[1])] + list(self.trail)
        if self.person is not None and (not self.trail or self.trail[-1] != self.person):
            pts.append(self.person)
        return pts

    @staticmethod
    def path_length(pts):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))

    @staticmethod
    def point_along(pts, dist):
        for a, b in zip(pts, pts[1:]):
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            if seg >= dist and seg > 1e-9:
                t = dist / seg
                return a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            dist -= seg
        return pts[-1]

    def turn(self, angle_error, dt):
        """Outer heading loop: heading error -> angular velocity setpoint."""
        if abs(angle_error) < math.radians(self.p['heading_tolerance']):
            self.heading_pid.reset()
            return 0.0
        limit = self.p['max_angular_vel']
        return self.heading_pid.update(angle_error, dt, -limit, limit)

    def stop_linear(self):
        self.linear_pid.reset()
        return 0.0

    def compute_command(self, pose, now, front_clearance=float('inf')):
        """Outer loop. Return (linear, angular, status, too_close) setpoints."""
        dt = 0.0 if self.last_command_time is None else now - self.last_command_time
        self.last_command_time = now
        if dt > 0.5:
            # Robot was stopped from outside (emergency, no data): start fresh
            self.linear_pid.reset()
            self.heading_pid.reset()
        if self.person is None:
            self.linear_pid.reset()
            self.heading_pid.reset()
            return 0.0, 0.0, 'Waiting for person position', False
        self.prune_trail(pose)
        visible = self.person_visible(now)
        px, py = to_robot_frame(pose, self.person)
        direct_dist = math.hypot(px, py)

        if visible and direct_dist < self.p['min_safe_distance']:
            self.heading_pid.reset()
            return self.stop_linear(), 0.0, f'Person too close: {direct_dist:.2f}m', True

        pts = self.path_points(pose)
        path_len = self.path_length(pts)
        stop_dist = self.p['follow_distance'] if visible else self.p['lost_approach_distance']
        remaining = path_len - stop_dist

        if remaining <= 0.0 and not visible:
            # Reached the last seen position: stop and look for the person
            v = self.stop_linear()
            if self.person_heading is None:
                self.heading_pid.reset()
                return v, 0.0, 'Person lost - waiting', False
            # The heading is measured on the last trail points, so it lags
            # behind a person who was turning: look further into the turn
            target = self.person_heading + self.person_turn_dir * math.radians(self.p['search_extra_angle'])
            err = wrap_angle(target - pose[2])
            return v, self.turn(err, dt), 'Person lost - looking where they went', False

        lookahead = min(self.p['lookahead_distance'], max(path_len, 1e-6))
        cx, cy = to_robot_frame(pose, self.point_along(pts, lookahead))
        alpha = math.atan2(cy, cx)

        # Speed scaling for sharp turns and obstacles (1.0 = no scaling)
        if abs(alpha) > math.radians(self.p['rotate_in_place_angle']):
            scale = 0.0
        else:
            scale = max(0.0, math.cos(alpha))
        status = 'Following trail' if visible else 'Person lost - going to last position'
        if front_clearance < self.p['obstacle_slow_dist']:
            span = self.p['obstacle_slow_dist'] - self.p['obstacle_stop_dist']
            scale *= max(0.0, (front_clearance - self.p['obstacle_stop_dist']) / span)
            status += f' | obstacle {front_clearance:.2f}m'

        # Outer distance loop. Do not integrate while the speed is held back,
        # otherwise the integral winds up and the robot jumps forward later.
        # Inside the follow distance the error is negative: P slows the robot
        # down and the integral leaks away instead of being cut to zero, so the
        # robot does not brake hard every time it touches the follow distance.
        if remaining <= 0.0:
            self.linear_pid.integral *= self.p['linear_i_leak']
        v = self.linear_pid.update(remaining, dt, 0.0, self.p['max_linear_vel'],
                                   integrate=scale > 0.9 and remaining > 0.0) * scale
        if remaining > 0.0 or v > 0.0:
            w = self.turn(alpha, dt)
        else:
            # Standing inside the follow distance: face the person
            w = self.turn(math.atan2(py, px), dt)
            status = 'Holding distance'
        return v, w, status, False


class VelocityLoop:
    """Inner loop: correct the command so the measured velocity follows the setpoint."""

    def __init__(self, p):
        self.p = p
        self.linear_pid = PID(p['vel_linear_kp'], p['vel_linear_ki'], 0.0, p['vel_linear_limit'])
        self.angular_pid = PID(p['vel_angular_kp'], p['vel_angular_ki'], 0.0, p['vel_angular_limit'])
        self.last_time = None

    def reset(self):
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.last_time = None

    def update(self, v_ref, w_ref, v_meas, w_meas, now):
        if v_ref == 0.0 and w_ref == 0.0:
            # Stop request: stop at once, no correction left over
            self.reset()
            return 0.0, 0.0
        dt = 0.0 if self.last_time is None else now - self.last_time
        self.last_time = now
        lim_v = self.p['vel_linear_limit']
        lim_w = self.p['vel_angular_limit']
        # Feedforward (the setpoint) + PI correction on the velocity error
        v_cmd = v_ref + self.linear_pid.update(v_ref - v_meas, dt, -lim_v, lim_v)
        w_cmd = w_ref + self.angular_pid.update(w_ref - w_meas, dt, -lim_w, lim_w)
        if v_ref == 0.0:
            v_cmd = 0.0
            self.linear_pid.reset()
        v_cmd = max(0.0, min(self.p['max_linear_vel'] + lim_v, v_cmd))
        max_w = self.p['max_angular_vel'] + lim_w
        w_cmd = max(-max_w, min(max_w, w_cmd))
        return v_cmd, w_cmd


class PersonPathFollower(Node):
    def __init__(self):
        super().__init__('person_path_follower')
        params = {name: self.declare_parameter(name, value).value
                  for name, value in DEFAULT_PARAMS.items()}
        odom_topic = self.declare_parameter('odom_topic', '/agv_controller/odom').value
        scan_topic = self.declare_parameter('scan_topic', '/scan').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', 'agv_controller/cmd_vel').value
        imu_topic = self.declare_parameter('imu_topic', '/imu/out').value
        self.p = params
        self.follower = TrailFollower(params)
        self.velocity_loop = VelocityLoop(params)

        # Inputs
        self.pose = None
        self.pose_time = None
        self.measured_v = 0.0
        self.measured_w = 0.0
        self.imu_yaw_rate = None
        self.imu_time = None
        self.detector_state = 'IDLE'
        self.last_tracking_time = None
        self.clusters_odom = []
        self.clusters_time = None
        self.front_clearance = float('inf')
        self.emergency_stop = False
        self.speed_limit = 1.0
        self.too_close_since = None
        self.has_sent_reset = False
        self.last_log_time = 0.0
        self.last_path_pub = 0.0

        self.create_subscription(PersonTracking, '/person/tracking', self.tracking_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        if params['use_velocity_loop'] and params['use_imu_yaw_rate']:
            self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)
        self.create_subscription(Float32, '/speed_limit_factor', self.speed_limit_callback, 10)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.reset_pub = self.create_publisher(Bool, '/detector/reset', 10)
        self.trail_pub = self.create_publisher(Path, '/person_follower/trail', 10)

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f"Person path follower started (follow {params['follow_distance']}m, "
            f"lookahead {params['lookahead_distance']}m, "
            f"velocity loop {'on' if params['use_velocity_loop'] else 'off'})")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.pose_time = time.time()
        self.measured_v = msg.twist.twist.linear.x
        self.measured_w = msg.twist.twist.angular.z

    def imu_callback(self, msg):
        self.imu_yaw_rate = msg.angular_velocity.z
        self.imu_time = time.time()

    def measured_velocity(self, now):
        w = self.measured_w
        if self.imu_time is not None and now - self.imu_time < 0.2:
            w = self.imu_yaw_rate
        return self.measured_v, w

    def tracking_callback(self, msg):
        now = time.time()
        if msg.state_name != self.detector_state:
            self.get_logger().info(f'Detector state: {self.detector_state} -> {msg.state_name}')
            if msg.state_name in ('IDLE', 'DETECTING') or self.detector_state in ('IDLE', 'DETECTING'):
                self.follower.reset()
        self.detector_state = msg.state_name
        self.last_tracking_time = now

        if (self.pose is None or not msg.following_state or
                msg.distance <= 0.0 or msg.distance > self.p['max_person_range']):
            return
        obs = self.follower.camera_observation(self.pose, msg.distance, msg.angle)
        if self.clusters_time is not None and now - self.clusters_time < 0.3:
            snapped = self.follower.match_clusters(self.clusters_odom, obs, self.p['camera_snap_radius'])
            if snapped is not None:
                obs = snapped
        self.follower.update_person(obs, now, from_camera=True)

    def scan_callback(self, msg):
        clusters, self.front_clearance = process_scan(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max, self.p)
        if self.pose is None:
            return
        now = time.time()
        self.clusters_odom = [(*self.follower.lidar_to_odom(self.pose, (cx, cy)), n)
                              for cx, cy, n in clusters]
        self.clusters_time = now

        if self.detector_state in ('FOLLOWING', 'STOPPING') and self.follower.lidar_tracking_allowed(now):
            match = self.follower.match_clusters(
                self.clusters_odom, self.follower.person, self.p['lidar_track_radius'])
            if match is not None:
                self.follower.update_person(match, now, from_camera=False)

    def emergency_callback(self, msg):
        if msg.data and not self.emergency_stop:
            self.get_logger().error('EMERGENCY STOP!!!')
        self.emergency_stop = msg.data

    def speed_limit_callback(self, msg):
        self.speed_limit = msg.data

    def compute_command(self, now):
        if self.emergency_stop:
            return 0.0, 0.0, 'Emergency stop'
        if self.pose_time is None or now - self.pose_time > 0.5:
            return 0.0, 0.0, 'No odometry'
        if self.last_tracking_time is None or now - self.last_tracking_time > 0.5:
            return 0.0, 0.0, 'No detector data'
        if self.detector_state not in ('FOLLOWING', 'STOPPING'):
            return 0.0, 0.0, f'Detector: {self.detector_state}'

        v, w, status, too_close = self.follower.compute_command(self.pose, now, self.front_clearance)
        if too_close:
            if self.too_close_since is None:
                self.too_close_since = now
                self.get_logger().warn(status)
            elif now - self.too_close_since > self.p['too_close_delay'] and not self.has_sent_reset:
                self.reset_pub.publish(Bool(data=True))
                self.has_sent_reset = True
                self.follower.reset()
                self.get_logger().warn('Person too close timeout - reset detector')
        else:
            self.too_close_since = None
            self.has_sent_reset = False
        return v * self.speed_limit, w * self.speed_limit, status

    def control_loop(self):
        now = time.time()
        v, w, status = self.compute_command(now)
        if self.p['use_velocity_loop']:
            if self.emergency_stop:
                self.velocity_loop.reset()
                v, w = 0.0, 0.0
            else:
                v, w = self.velocity_loop.update(v, w, *self.measured_velocity(now), now)
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)
        self.cmd_vel_pub.publish(cmd)

        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            self.get_logger().info(
                f'{status} | trail: {len(self.follower.trail)} pts | vel: [{v:.2f}, {w:.2f}]')
        if now - self.last_path_pub > 0.2:
            self.last_path_pub = now
            self.publish_trail()

    def publish_trail(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        for x, y in self.follower.trail:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.trail_pub.publish(path)

    def destroy_node(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'
        self.cmd_vel_pub.publish(cmd)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
