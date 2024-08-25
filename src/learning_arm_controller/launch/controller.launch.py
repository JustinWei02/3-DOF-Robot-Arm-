from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_description = ParameterValue(
        Command(
            [
                "xacro ", 
                os.path.join(get_package_share_directory("learning_arm_description"), "urdf", "learningarm.urdf.xacro")
            ]
        ),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])
