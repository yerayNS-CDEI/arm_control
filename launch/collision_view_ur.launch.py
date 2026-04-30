# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="arm_control",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    # End-effector cylinder parameters
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_cylinder_length",
            default_value="0.15",
            description="Length of the end-effector cylinder.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_cylinder_radius",
            default_value="0.035",
            description="Radius of the end-effector cylinder.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sensors_offset",
            default_value="0.15",
            description="Offset for sensors mounting position.",
        )
    )
    # Mode argument: arm (arm only) or full (mobile manipulator)
    declared_arguments.append(
        DeclareLaunchArgument(
            "mode",
            default_value="arm",
            description="Collision robot mode: 'arm' for arm-only URDF, 'full' for mobile manipulator URDF",
            choices=["arm", "full"],
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    mode = LaunchConfiguration("mode")
    # End-effector parameters
    ee_cylinder_length = LaunchConfiguration("ee_cylinder_length")
    ee_cylinder_radius = LaunchConfiguration("ee_cylinder_radius")
    sensors_offset = LaunchConfiguration("sensors_offset")

    # Conditional URDF selection based on mode
    # For 'arm' mode: use ur.urdf.xacro from arm_control package
    # For 'full' mode: use mobile_manipulator.urdf.xacro from navi_wall package
    
    def launch_setup(context, *args, **kwargs):
        mode_value = LaunchConfiguration('mode').perform(context)
        ur_type_value = LaunchConfiguration('ur_type').perform(context)
        tf_prefix_value = LaunchConfiguration('tf_prefix').perform(context)
        ee_cylinder_length_value = LaunchConfiguration('ee_cylinder_length').perform(context)
        ee_cylinder_radius_value = LaunchConfiguration('ee_cylinder_radius').perform(context)
        sensors_offset_value = LaunchConfiguration('sensors_offset').perform(context)
        
        print(f"[collision_view_ur DEBUG] mode_value = '{mode_value}'")
        print(f"[collision_view_ur DEBUG] tf_prefix_value = '{tf_prefix_value}'")
        
        if mode_value == 'full':
            # Full mobile manipulator URDF
            navi_wall_share = FindPackageShare('navi_wall').perform(context)
            urdf_file = PathJoinSubstitution([navi_wall_share, 'navi_wall_description', 'description', 'mobile_manipulator.urdf.xacro']).perform(context)
            xacro_command = [
                FindExecutable(name='xacro').perform(context),
                ' ', urdf_file,
                ' simulation:=false',
                ' use_mock_hardware:=false',
                ' ur_type:=', ur_type_value,
                ' tf_prefix:=', tf_prefix_value,
                ' ee_cylinder_length:=', ee_cylinder_length_value,
                ' ee_cylinder_radius:=', ee_cylinder_radius_value,
                ' sensors_offset:=', sensors_offset_value,
            ]
        else:
            # Arm-only URDF
            description_package_value = LaunchConfiguration('description_package').perform(context)
            description_file_value = LaunchConfiguration('description_file').perform(context)
            safety_limits_value = LaunchConfiguration('safety_limits').perform(context)
            safety_pos_margin_value = LaunchConfiguration('safety_pos_margin').perform(context)
            safety_k_position_value = LaunchConfiguration('safety_k_position').perform(context)
            
            arm_control_share = FindPackageShare(description_package_value).perform(context)
            urdf_file = PathJoinSubstitution([arm_control_share, 'urdf', description_file_value]).perform(context)
            xacro_command = [
                FindExecutable(name='xacro').perform(context),
                ' ', urdf_file,
                ' safety_limits:=', safety_limits_value,
                ' safety_pos_margin:=', safety_pos_margin_value,
                ' safety_k_position:=', safety_k_position_value,
                ' name:=ur',
                ' ur_type:=', ur_type_value,
                ' tf_prefix:=', tf_prefix_value,
                ' ee_cylinder_length:=', ee_cylinder_length_value,
                ' ee_cylinder_radius:=', ee_cylinder_radius_value,
                ' sensors_offset:=', sensors_offset_value,
            ]
        
        robot_description_content = Command([''.join(xacro_command)])
        robot_description = {
            'robot_description': ParameterValue(value=robot_description_content, value_type=str)
        }

        rviz_config_file = PathJoinSubstitution(
            [FindPackageShare('arm_control'), 'rviz', 'view_collision_robot.rviz']
        ).perform(context)

        # Add frame_prefix to robot_description so all TF frames get /collision/ prefix
        robot_description_with_prefix = {'robot_description': robot_description['robot_description'],
                                          'frame_prefix': 'collision/'}

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='collision',
            output='both',
            parameters=[robot_description_with_prefix],
        )
        
        # Note: joint_state_publisher is NOT included - the collision checking service
        # publishes joint states only when checking a configuration, so the robot
        # visualization will stay at the last tested configuration
        
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config_file],
        )

        nodes_to_start = [
            robot_state_publisher_node,
            rviz_node,
        ]
        return nodes_to_start

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
