from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    markers_node = Node(
        package='simulation',
        executable='markers',
        output='screen'
    )
    tf_broadcaster_node = Node(
        package='simulation',
        executable='tf_broadcaster',
        output='screen'
    )
    # ld.add_action(markers_node) #TODO: markers are not deleted
    ld.add_action(tf_broadcaster_node)
    return ld