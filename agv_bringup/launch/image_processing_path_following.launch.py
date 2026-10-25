from launch import LaunchDescription
from launch_ros.actions import Node

# Same as image_processing.launch.py, but the robot follows the person's path
# (breadcrumb trail + LiDAR) instead of steering straight at the person.
def generate_launch_description():
    path_follower_node = Node(
        package='agv_image_processing',
        executable='person_path_follower',
        name='person_path_follower',
        output='screen',
        parameters=[{
            'follow_distance': 1.0,
            'lookahead_distance': 0.5,
            'max_linear_vel': 0.38,
            'max_angular_vel': 0.43,
            # Outer loop (position -> velocity setpoint)
            'linear_kp': 0.5,
            'linear_ki': 0.2,
            'angular_kp': 1.2,
            'angular_kd': 0.1,
            # Inner loop (velocity setpoint vs wheel odometry -> correction)
            'use_velocity_loop': True,
            'vel_linear_kp': 0.3,
            'vel_linear_ki': 1.0,
            'vel_angular_kp': 0.3,
            'vel_angular_ki': 1.0,
            # Keep False until the MPU6050 gyro scale in mpu6050_driver.py is fixed
            'use_imu_yaw_rate': False,
        }]
    )

    safety_control_node = Node(
        package='agv_image_processing',
        executable='safety_ultrasonic',
        name='safety_ultrasonic',
        output='screen'
    )

    range_detection_node = Node(
        package='agv_image_processing',
        executable='range_detection_with_ultrasonic',
        name='range_detector',
        output='screen',
        # Longer wait before giving up, so the robot has time to reach the
        # corner where the person was last seen
        parameters=[{'stop_delay': 8.0}]
    )

    return LaunchDescription([
        path_follower_node,
        safety_control_node,
        range_detection_node
    ])
