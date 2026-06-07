import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    rplidar_a1 = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("agv_package"),
            "launch",
            "rp_lidar_a1_AGV.launch.py"
        ),
    )
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("agv_package"), "urdf", "agv.urdf.xacro"),
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("agv_package"),
            "config",
            "slam_lifelong.yaml"
        )
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)
    
    slam_config = LaunchConfiguration("slam_config")
    
    imu_node = Node(
        package="agv_package",
        executable="imu.py",
        name="imu_driver",
        output="screen"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("agv_package"), "config", "ekf_with_imu.yaml")]
    )

    lifelong_mapping = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",  
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": False}
        ],
        remappings=[
            ('/odom', '/odometry/filtered')
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory("agv_package"),
                        "rviz",
                        "AGV_config.rviz"
        )],
        output="screen",
        

    
    )

    return LaunchDescription([
        rplidar_a1,
        model_arg,
        slam_config_arg,
        imu_node,
        robot_localization,
        robot_state_publisher_node,
        joint_state_publisher,  
        lifelong_mapping,  
        rviz_node,
    ])
