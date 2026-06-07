import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    #Declare parameters to change in terminal
    use_async_mode_arg = DeclareLaunchArgument(
        "async_mode",
        default_value="true"
    )
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("agv_package"),"urdf","agv.urdf.xacro"),
    )
    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value =os.path.join(get_package_share_directory("agv_package"),"config","slam_lifelong.yaml")
    )
    
    #Run Lidar
    rplidar_a1 = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("agv_package"),
            "launch",
            "rp_lidar_a1_AGV.launch.py"
        ),
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),value_type=str)
    use_async = LaunchConfiguration("async_mode")
    slam_config = LaunchConfiguration("slam_config")
    
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
        parameters=[os.path.join(get_package_share_directory("agv_package"), "config", "ekf_without_imu.yaml")]
    )
    
    slam_toolbox_async = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node", #Change to async to test 
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config,{"use_sim_time": False}],
        remappings=[('/odom', '/odometry/filtered')],
        condition=IfCondition(use_async)
    )

    slam_toolbox_sync = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node", 
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config,{"use_sim_time": False}],
        remappings=[('/odom', '/odometry/filtered')],
        condition=UnlessCondition(use_async)
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory("agv_package"),"rviz","AGV_config.rviz")],
        output="screen",
    )

    return LaunchDescription([
        use_async_mode_arg,
        model_arg,
        slam_config_arg,
        rplidar_a1,
        robot_localization,
        robot_state_publisher_node,
        joint_state_publisher,
        slam_toolbox_async,
        slam_toolbox_sync,
        rviz_node
    ])
