import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    mode = LaunchConfiguration("mode")
    namespace_arm = LaunchConfiguration("namespace_arm")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    mode_value = context.perform_substitution(mode).strip().lower()
    arm_namespace_value = context.perform_substitution(namespace_arm).strip().strip("/")
    controller_namespace = f"/{arm_namespace_value}" if arm_namespace_value else ""
    joint_states_topic = (
        f"{controller_namespace}/joint_states" if controller_namespace else "/joint_states"
    )

    xacro_executable = PathJoinSubstitution([FindExecutable(name="xacro")])
    if mode_value == "full":
        urdf_file = PathJoinSubstitution(
            [
                FindPackageShare("navi_wall"),
                "navi_wall_description/description",
                "mobile_manipulator.urdf.xacro",
            ]
        )
        srdf_file = PathJoinSubstitution(
            [FindPackageShare("arm_control"), "srdf", "mobile_manipulator.srdf.xacro"]
        )
        robot_name = "mobile_manipulator"
    elif mode_value == "arm":
        urdf_file = PathJoinSubstitution(
            [FindPackageShare("arm_control"), "urdf", "ur.urdf.xacro"]
        )
        srdf_file = PathJoinSubstitution(
            [FindPackageShare("arm_control"), "srdf", "arm_control.srdf.xacro"]
        )
        robot_name = "ur"
    else:
        raise RuntimeError(
            f"Mode not recognized: '{mode_value}'. Please select 'full' or 'arm'."
        )
    rviz_file = PathJoinSubstitution([FindPackageShare("arm_control"), "rviz", "moveit.rviz"])

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_command = [
        xacro_executable,
        " ",
        urdf_file,
        " ",
        "robot_ip:=0.0.0.0",
        " ",
        "joint_limit_params:=",
        joint_limit_params,
        " ",
        "kinematics_params:=",
        kinematics_params_file,
        " ",
        "physical_params:=",
        physical_params,
        " ",
        "visual_params:=",
        visual_params,
        " ",
        "safety_limits:=true",
        " ",
        "safety_pos_margin:=0.15",
        " ",
        "safety_k_position:=20",
        " ",
        "ur_type:=",
        ur_type,
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "use_fake_hardware:=",
        use_fake_hardware,
        " ",
        "fake_sensor_commands:=false",
        " ",
        "headless_mode:=true",
        " ",
        "sim_gazebo:=false",
        " ",
        "sim_ignition:=false",
        " ",
    ]
    if mode_value == "arm":
        robot_description_command.extend(["name:=ur", " "])
    else:
        robot_description_command.extend(["simulation:=false", " "])

    robot_description_content = Command(robot_description_command)
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command(
        [
            xacro_executable,
            " ",
            srdf_file,
            " ",
            "name:=",
            robot_name,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    kinematics_yaml = load_yaml("arm_control", os.path.join("config", "moveit", "kinematics.yaml"))
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    rviz_kinematics = kinematics_yaml
    robot_description_planning_data = load_yaml(
        "arm_control", os.path.join("config", "moveit", "joint_limits.yaml")
    )
    robot_description_planning_data.update(
        load_yaml("arm_control", os.path.join("config", "moveit", "pilz_cartesian_limits.yaml"))
    )
    robot_description_planning = {"robot_description_planning": robot_description_planning_data}

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(
        load_yaml("arm_control", os.path.join("config", "moveit", "ompl_planning.yaml"))
    )
    pilz_planning_pipeline_config = {
        "pilz_industrial_motion_planner": load_yaml(
            "arm_control",
            os.path.join("config", "moveit", "pilz_industrial_motion_planner_planning.yaml"),
        )
    }
    pilz_capabilities = pilz_planning_pipeline_config["pilz_industrial_motion_planner"].get(
        "capabilities", ""
    )
    planning_pipeline_settings = {
        "planning_pipelines": ["move_group", "pilz_industrial_motion_planner"],
        "default_planning_pipeline": "move_group",
    }

    controllers_yaml = load_yaml("arm_control", os.path.join("config", "moveit", "controllers.yaml"))

    namespaced_controllers_yaml = {"controller_names": []}
    for controller_name in controllers_yaml["controller_names"]:
        namespaced_name = (
            f"{controller_namespace}/{controller_name}" if controller_namespace else controller_name
        )
        namespaced_controllers_yaml["controller_names"].append(namespaced_name)
        namespaced_controllers_yaml[namespaced_name] = controllers_yaml[controller_name]

    moveit_controllers = {
        "moveit_simple_controller_manager": namespaced_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    common_parameters = [
        robot_description,
        robot_description_semantic,
        {"publish_robot_description_semantic": publish_robot_description_semantic},
        robot_description_kinematics,
        robot_description_planning,
        planning_pipeline_settings,
        ompl_planning_pipeline_config,
        pilz_planning_pipeline_config,
        {"capabilities": pilz_capabilities},
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        {"use_sim_time": use_sim_time},
        warehouse_ros_config,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_parameters,
        remappings=[('joint_states', joint_states_topic)],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {"publish_robot_description_semantic": publish_robot_description_semantic},
            rviz_kinematics,
            planning_pipeline_settings,
            ompl_planning_pipeline_config,
            pilz_planning_pipeline_config,
            {"move_group.planning_plugin": "ompl_interface/OMPLPlanner"},
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur10e"),
            DeclareLaunchArgument(
                "mode",
                default_value="arm",
                description="Launch mode full|arm",
                choices=["full", "arm"],
            ),
            DeclareLaunchArgument("namespace_arm", default_value=""),
            DeclareLaunchArgument("tf_prefix", default_value="arm_"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "publish_robot_description_semantic",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            ),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("arm_control"), "config", "my_robot_calibration.yaml"]
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
