from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():

    # TF
    tf_node=Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='static_tf_pub',  # You can name your node for easier identification
      arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],  # Example transformation
      # Arguments order: x, y, z, roll, pitch, yaw, frame_id, child_frame_id
      # These values are just examples; replace them with your actual transformation data
      output='screen'
    )

    # NAV 2
    # 2D Costmap
    nav2_costmap_path_arg = DeclareLaunchArgument(
        'costmap_yaml',
        default_value=str(Path(get_package_share_directory("radar_layer")) / "config" / "demo_costmap.yaml"),
        description='Path to the costmap ros params YAML file.'
    )
    nav2_costmap_path = LaunchConfiguration('costmap_yaml')
    nav2_2d_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap_node',
        output='screen',
        parameters=[nav2_costmap_path],
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # Lifecycle Manager
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[{'autostart': True},
                    { 'bond_timeout': 0.0 },
                    {'node_names': ['costmap/costmap']}])
    

    # Add actions
    ld = LaunchDescription([
      tf_node,
      nav2_lifecycle_manager,
      nav2_costmap_path_arg,
      nav2_2d_costmap
    ])

    return ld

