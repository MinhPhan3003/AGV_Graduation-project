from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node




def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("agv_description"),
                    "urdf",
                    "agv_description.urdf.xacro",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
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


    controller_manager = Node( 
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
             os.path.join(
                 get_package_share_directory("agv_controller"),
                 "config", 
                 "agv_controllers.yaml"
             )
        ]

    )
    
    imu_driver_node = Node(
         package="agv_firmware",
         executable="mpu6050_driver.py"
     )

    return LaunchDescription([
        controller_manager,
        robot_state_publisher_node,
        joint_state_publisher,
        imu_driver_node,

    ])
