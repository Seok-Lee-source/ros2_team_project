import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = os.path.join(get_package_share_directory("urdf_project"))
    xacro_file = os.path.join(pkg_path, "urdf", "xycar.xacro")
    robot_description = xacro.process_file(xacro_file)
    params = {"robot_description": robot_description.toxml(), "use_sim_time": use_sim_time}


    launch_actions = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="use sim time"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[params],
        ),
    ]

   #spon robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description",
        "-entity", "with_robot",
        "-x", "16.2",
        "-y", "10.29",
        "-z", "0.25"
        ],
        output="screen",
    )

    #launch_action include spawn_entity
    launch_actions.append(spawn_entity)

    #launch them all
    return LaunchDescription(launch_actions)