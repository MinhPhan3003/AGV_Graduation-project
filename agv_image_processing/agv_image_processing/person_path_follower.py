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
"""
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from agv_interfaces.msg import PersonTracking

DEFAULT_PARAMS = {
    # Following
    'follow_distance': 1.0,          # desired distance along the trail [m]
    'min_safe_distance': 0.55,       # straight-line distance to stop [m]
    'too_close_delay': 5.0,          # too close this long -> reset detector [s]
    'lost_approach_distance': 0.3,   # stop this far from last seen position [m]
    # Trail
    'breadcrumb_spacing': 0.15,
    'waypoint_reached_dist': 0.25,
    'lookahead_distance': 0.5,       # smaller = follows corners more tightly
    'max_trail_points': 300,
    # Velocity
    'max_linear_vel': 0.38,
    'max_angular_vel': 0.43,
    'linear_kp': 0.5,
    'angular_kp': 1.2,
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
        self.reset()

    def reset(self):
        self.trail.clear()
        self.person = None
        self.person_heading = None
        self.last_person_update = None
        self.last_camera_update = None
        self.jump_count = 0

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
        else:
            lx, ly = self.trail[-1]
            if math.hypot(self.person[0] - lx, self.person[1] - ly) >= self.p['breadcrumb_spacing']:
                self.trail.append(self.person)
                ref = self.trail[-4] if len(self.trail) >= 4 else self.trail[0]
                if math.hypot(self.person[0] - ref[0], self.person[1] - ref[1]) > 0.1:
                    self.person_heading = math.atan2(self.person[1] - ref[1],
                                                     self.person[0] - ref[0])

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

    def turn(self, angle_error):
        if abs(angle_error) < math.radians(self.p['heading_tolerance']):
            return 0.0
        w = self.p['angular_kp'] * angle_error
        return max(-self.p['max_angular_vel'], min(self.p['max_angular_vel'], w))

    def compute_command(self, pose, now, front_clearance=float('inf')):
        """Return (linear, angular, status, too_close)."""
        if self.person is None:
            return 0.0, 0.0, 'Waiting for person position', False
        self.prune_trail(pose)
        visible = self.person_visible(now)
        px, py = to_robot_frame(pose, self.person)
        direct_dist = math.hypot(px, py)

        if visible and direct_dist < self.p['min_safe_distance']:
            return 0.0, 0.0, f'Person too close: {direct_dist:.2f}m', True

        pts = self.path_points(pose)
        path_len = self.path_length(pts)
        stop_dist = self.p['follow_distance'] if visible else self.p['lost_approach_distance']
        remaining = path_len - stop_dist

        if remaining <= 0.0:
            if visible:
                return 0.0, self.turn(math.atan2(py, px)), 'Holding distance', False
            if self.person_heading is None:
                return 0.0, 0.0, 'Person lost - waiting', False
            err = wrap_angle(self.person_heading - pose[2])
            return 0.0, self.turn(err), 'Person lost - looking where they went', False

        lookahead = min(self.p['lookahead_distance'], path_len)
        cx, cy = to_robot_frame(pose, self.point_along(pts, lookahead))
        alpha = math.atan2(cy, cx)

        v = min(self.p['max_linear_vel'], self.p['linear_kp'] * remaining)
        if abs(alpha) > math.radians(self.p['rotate_in_place_angle']):
            v = 0.0
        else:
            v *= max(0.0, math.cos(alpha))
        w = self.turn(alpha)

        status = 'Following trail' if visible else 'Person lost - going to last position'
        if front_clearance < self.p['obstacle_slow_dist']:
            span = self.p['obstacle_slow_dist'] - self.p['obstacle_stop_dist']
            factor = max(0.0, (front_clearance - self.p['obstacle_stop_dist']) / span)
            v *= factor
            status += f' | obstacle {front_clearance:.2f}m'
        return v, w, status, False


class PersonPathFollower(Node):
    def __init__(self):
        super().__init__('person_path_follower')
        params = {name: self.declare_parameter(name, value).value
                  for name, value in DEFAULT_PARAMS.items()}
        odom_topic = self.declare_parameter('odom_topic', '/agv_controller/odom').value
        scan_topic = self.declare_parameter('scan_topic', '/scan').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', 'agv_controller/cmd_vel').value
        self.p = params
        self.follower = TrailFollower(params)

        # Inputs
        self.pose = None
        self.pose_time = None
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
        self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)
        self.create_subscription(Float32, '/speed_limit_factor', self.speed_limit_callback, 10)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.reset_pub = self.create_publisher(Bool, '/detector/reset', 10)
        self.trail_pub = self.create_publisher(Path, '/person_follower/trail', 10)

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f"Person path follower started (follow {params['follow_distance']}m, "
            f"lookahead {params['lookahead_distance']}m)")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.pose_time = time.time()

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
