from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot_names = [
        'Giskard',
        'BB8',
        'Daneel',
        'Jander',
        'C3PO'
    ]

    node_list = []

    for name in robot_names:

        node_list.append(Node(
            package='my_cpp_pkg',
            executable='robot_news_station',
            name=f'robot_news_station_{name.lower()}',
            parameters=([{'robot_name': name}])
        ))

    smartphone = Node(
        package='my_cpp_pkg',
        executable='smartphone'
    )

    for node in node_list:
        ld.add_action(node)
    ld.add_action(smartphone)

    return ld
