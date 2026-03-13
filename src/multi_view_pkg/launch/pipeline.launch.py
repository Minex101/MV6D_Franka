import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Vision Node
    vision_node = Node(
        package='multi_view_pkg',
        executable='vision4',
        name='vision_node',
        output='screen'
    )

    # Movement Node
    movement_node = Node(
        package='multi_view_pkg',
        executable='movement4',
        name='movement_node',
        output='screen',
        parameters=[{
            'urdf_path': "/home/affan/Documents/FY_Project/isaac_sim_assets/franka.urdf"
        }]
    )

    # Fusion Node
    fusion_node = Node(
        package='multi_view_pkg',
        executable='fusion4',
        name='fusion_node',
        output='screen'
    )

    return LaunchDescription([
        vision_node,
        movement_node,
        fusion_node
    ])