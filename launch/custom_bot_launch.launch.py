import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    
    package_dir = get_package_share_directory('webots_diffdrive')
    robot_description_path = os.path.join(package_dir, 'resource', 'custom_robot.urdf')
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'custom_world.wbt')
    )

    robot_drivers = []
    for i in range (1,3):
        my_robot_driver = WebotsController(
            robot_name=f'custom_robot_{i}',
            parameters=[
                {'robot_description': robot_description_path},
            ],
            remappings=[
                ('cmd_vel', f'/cb{i}/cmd_vel')
            ],
        )
        robot_drivers.append(my_robot_driver)

    return LaunchDescription([
        webots,
        *robot_drivers,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])