import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Command-line arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    # UR5 configuration
    ur_type = "ur5"
    description_package = "ur_description"
    moveit_config_package = "ur_moveit_config"
    
    # Get package paths
    ur_description_path = get_package_share_directory(description_package)
    ur_moveit_config_path = get_package_share_directory(moveit_config_package)
    
    # Generate robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([ur_description_path, "urdf", "ur.urdf.xacro"]),
            " ",
            "ur_type:=", ur_type,
            " ",
            "prefix:=", '""',
            " ",
            "use_fake_hardware:=true",
            " ",
            "safety_limits:=false",
            " ",
            "robot_ip:=dummy_value",
            " ",
            "joint_limit_params:=",
            PathJoinSubstitution([ur_description_path, "config", ur_type, "joint_limits.yaml"]),
            " ",
            "kinematics_params:=",
            PathJoinSubstitution([ur_description_path, "config", ur_type, "default_kinematics.yaml"]),
            " ",
            "physical_params:=",
            PathJoinSubstitution([ur_description_path, "config", ur_type, "physical_parameters.yaml"]),
            " ",
            "visual_params:=",
            PathJoinSubstitution([ur_description_path, "config", ur_type, "visual_parameters.yaml"]),
            " ",
            "name:=ur",
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
        ]
    )

    # Generate robot description semantic
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([ur_moveit_config_path, "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=ur",
            " ",
            "prefix:=", '""',
        ]
    )

    # MoveIt parameters
    moveit_config = {
        "robot_description": robot_description_content,
        "robot_description_semantic": robot_description_semantic_content,
        # Explicit kinematics configuration
        "robot_description_kinematics": {
            "manipulator": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05
            }
        },
        "planning_scene_monitor": {
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            # Corrected frame names based on your TF tree
            "octomap_frame": "world",
            "octomap_resolution": 0.05,
        },
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
        "trajectory_execution": {
            "moveit_manage_controllers": True,
            "allowed_execution_duration_scaling": 1.2,
            "allowed_goal_duration_margin": 0.5,
            "allowed_start_tolerance": 0.01,
            "execution_duration_monitoring": False,
        },
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "controller_manager_name": "/controller_manager",
        # Explicit controller configuration
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": ["joint_trajectory_controller"],
            "joint_trajectory_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",  # ADDED ACTION NAMESPACE
                "default": True,
                "joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint"
                ]
            }
        },
        "use_sim_time": False  # Must be consistent across all nodes
    }

    # Load additional MoveIt configuration files
    def load_yaml(package_name, file_path):
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        try:
            with open(absolute_file_path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            print(f"Exception loading file {absolute_file_path}: {e}")
            return {}

    # Add OMPL planning configuration
    ompl_planning_yaml = load_yaml(
        "ur_moveit_config", "config/ompl_planning.yaml"
    )
    if ompl_planning_yaml:
        moveit_config["ompl"].update(ompl_planning_yaml.get("ompl", {}))

    # Add sensors configuration
    sensors_config = load_yaml(
        "ros2_manipulator_dynamics_planner", "config/sensors_3d.yaml"
    )
    if sensors_config:
        moveit_config.update(sensors_config)

    # Static TF for cameras - corrected frame names
    static_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera1_transform",
        arguments=[
            "--frame-id", "world",
            "--child-frame-id", "camera_1_link",
            "--x", "0.5", "--y", "0", "--z", "0.5",
            "--roll", "-1.57", "--pitch", "0", "--yaw", "-1.57"
        ]
    )
    
    static_2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera2_transform",
        arguments=[
            "--frame-id", "world",
            "--child-frame-id", "camera_2_link",
            "--x", "0.5", "--y", "0.2", "--z", "0.5",
            "--roll", "-1.57", "--pitch", "0", "--yaw", "-1.57"
        ]
    )

    return LaunchDescription([
        rviz_config_arg,
        static_1,
        static_2,
    ])