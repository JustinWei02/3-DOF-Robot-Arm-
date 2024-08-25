from launch import LaunchDescription 
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    
    learning_arm_description = get_package_share_directory("learning_arm_description")
    learning_arm_description_prefix = get_package_prefix("learning_arm_description")
    
    model_path=os.path.join(learning_arm_description, "models")
    model_path += os.pathsep + os.path.join(learning_arm_description_prefix, "share")
    

    gz_sim_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)
       
    model_arg=DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("learning_arm_description"), "urdf", "learningarm.urdf.xacro"),
        description="absolute path to the robot URDF file"
    )
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
    
    robot_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    

    #start_gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
    #    get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"        
    #)))
    
    # Start Gazebo with Bullet Featherstone plugin
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments=[
            ('gz_args', '-r -v empty.sdf --physics-engine gz-physics-bullet-featherstone-plugin')
        ]
    )

    spawn_robot=Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-entity", "learning_arm", "-topic", "robot_description"]
    )
    return LaunchDescription([
        gz_sim_resource_path,
        model_arg, 
        robot_state_publisher, 
        start_gazebo, 
        spawn_robot
    ])
    
