from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='niz_xvn_tsp',
            executable='tsp_node',
            parameters=[
                {"file_name" : "src/niz_xvn_tsp/csv/test2.csv"},
            ],
            output='screen',
        ),
    ])