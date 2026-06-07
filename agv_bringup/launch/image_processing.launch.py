from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    agv_controller_node = Node(
        package='agv_image_processing',
        executable='controller_agv_speed_ultrasonic',
        name='controller_with_ultrasonic',
        output='screen'
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
        output='screen'
            )

    return LaunchDescription([
        agv_controller_node,
        safety_control_node,
        range_detection_node
    ])

