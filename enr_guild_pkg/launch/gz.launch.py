from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set your package and URDF file name
    pkg_name = 'enr_guild_pkg'  # <-- change to your package
    urdf_file = 'diff_drive_bot.urdf'            # <-- change to your URDF file

    # Get full path to URDF
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        urdf_file
    )

    # Start Ignition Gazebo (Gazebo Sim)
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ])
    )

    # Publish robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # Spawn robot in Ignition Gazebo using ros_gz_sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim_launch,
        robot_state_publisher,
        spawn_entity
    ])
