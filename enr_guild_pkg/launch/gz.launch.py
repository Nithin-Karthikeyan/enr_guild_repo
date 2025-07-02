from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with empty world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        # Publish robot_state_publisher with your URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open('/home/nithin/enr_guild_ws/src/enr_guild_pkg/urdf/diff_drive_bot.urdf').read()}]
        ),
        # Spawn your robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', '/home/nithin/enr_guild_ws/src/enr_guild_pkg/urdf/diff_drive_bot.urdf'],
            output='screen'
        ),
    ])
