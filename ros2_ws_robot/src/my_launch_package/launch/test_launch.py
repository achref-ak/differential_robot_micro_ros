from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'agent_port',
            default_value='8888',
            description='Port for Micro-ROS agent communication'
        ),
        DeclareLaunchArgument(
            'host_ip',
            default_value='172.20.10.2',  # Change to your host IP
            description='Host IP address for WiFi communication'
        ),
        DeclareLaunchArgument(
            'camera_device',
            default_value='0',
            description='Camera device number (e.g., 0 for default camera, 1 for external)'
        ),
        
        # Micro-ROS Agent Node
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['udp4', '--port', LaunchConfiguration('agent_port')],
            parameters=[{
                'verbose': True,
                'discovery_port': LaunchConfiguration('agent_port'),
            }]
        ),
        
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',  # Output in current terminal
            prefix='xterm -e'
        ),
        
        Node(
            package='camera_py',
            executable='camera',
            name='camera',
            output='screen',  # Output in current terminal
            parameters=[{
                'camera_device': LaunchConfiguration('camera_device')
            }]
        )
    ])

