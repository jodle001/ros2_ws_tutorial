from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    ld = LaunchDescription()

    turtle_main = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )

    turtle_controller = Node(
        package='turtlesim_catch_them_all',
        executable='turtle_controller'
    )

    turtle_spawner = Node(
        package='turtlesim_catch_them_all',
        executable='turtle_spawner'
    )

    # Quit all processes when main window is closed
    quit_event = RegisterEventHandler(
        OnProcessExit(
            target_action=turtle_main,
            on_exit=[
                LogInfo(msg=(EnvironmentVariable(name='USER'),
                             ' closed the turtle window')),
                EmitEvent(event=Shutdown(
                    reason='Window closed'))
            ]
        )

    )

    ld.add_action(turtle_main)
    ld.add_action(turtle_controller)
    ld.add_action(turtle_spawner)
    ld.add_action(quit_event)

    return ld
