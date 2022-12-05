from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtleMain = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )

    turtleController = Node(
        package='turtlesim_catch_them_all',
        executable='turtle_controller'
    )

    turtleSpawner = Node(
        package='turtlesim_catch_them_all',
        executable='turtle_spawner'
    )
    
    ld.add_action(turtleMain)
    ld.add_action(turtleController)
    ld.add_action(turtleSpawner)

    return ld
