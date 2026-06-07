from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition



def noisy_controller(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))
    use_python = LaunchConfiguration("use_python")


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
    )
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False"
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.07"
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.402"
    )
    gear_ratio_arg = DeclareLaunchArgument(
        "gear_ratio",
        default_value="6.0"
    )


    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True"
    )

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005"
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02"
    )


    use_sim_time = LaunchConfiguration("use_sim_time")
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius") 
    wheel_separation = LaunchConfiguration("wheel_separation")
    gear_ratio = LaunchConfiguration("gear_ratio")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    joint_state_broadcaster_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments =[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    wheel_controller_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments =[
            "agv_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        condition = UnlessCondition(use_simple_controller)
    )

    simple_controller = GroupAction(
        condition = IfCondition(use_simple_controller),
        actions=[
            Node(
                package = "controller_manager",
                executable = "spawner",
                arguments =[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager",
                ]
            ),  


            Node(
                package = "agv_controller",
                executable = "simple_controller.py",
                parameters = [
                    {"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,
                    "gear_ratio": gear_ratio,
                    "use_sim_time": use_sim_time}
                ],
                condition = IfCondition(use_python)
            ),

            Node(
                package = "agv_controller",
                executable = "simple_controller",
                parameters = [
                    {"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,
                    "gear_ratio": gear_ratio,
                    "use_sim_time": use_sim_time}
                ],
                condition = UnlessCondition(use_python)
            )
        ]
    )





    return LaunchDescription([
        use_sim_time_arg,
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        gear_ratio_arg,
        use_simple_controller_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller,
    ])
