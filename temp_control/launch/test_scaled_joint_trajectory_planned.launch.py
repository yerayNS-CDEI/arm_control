from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument    
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    controller_config = PathJoinSubstitution(
        [FindPackageShare("ur_arm_control"), "config", "test_goal_planned_config.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "check_starting_point",
                default_value="false",
                description="Check if current joint state is within allowed starting limits"
            ),

            Node(
                package="ur_arm_control",
                executable="publisher_joint_trajectory_planned",
                name="publisher_joint_trajectory_planned",
                parameters=[
                    controller_config,
                    {"check_starting_point": LaunchConfiguration("check_starting_point")}
                ],
                output="screen",
            )
        ]
    )
