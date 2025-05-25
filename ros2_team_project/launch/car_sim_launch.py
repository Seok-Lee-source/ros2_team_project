from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_path = FindPackageShare('ros2_team_project').find('ros2_team_project')
    world_file = os.path.join(pkg_path, 'worlds', 'test_track.world')

    # gazebo_ros의 기본 launch 파일 포함 (world 파일 설정)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('gazebo_ros').find('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # robot_5.launch.py를 포함
    robot_5_launch= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('urdf_project').find('urdf_project'),
                'launch', 'robot_5.launch.py'
            )
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_5_launch
    ])
