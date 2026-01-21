from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Vision Node (PoseCollector_2)
    # Ensure this matches the entry_point name in your setup.py
    vision_node = Node(
        package='multi_view_pkg',
        executable='vision2',
        name='vision_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 2. Movement Node (MultiViewCoordinator)
    movement_node = Node(
        package='multi_view_pkg',
        executable='movement_node2',
        name='movement_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. Fusion Node (FusionNode)
    fusion_node = Node(
        package='multi_view_pkg',
        executable='fusion_node2',
        name='fusion_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        vision_node,
        movement_node,
        fusion_node
    ])