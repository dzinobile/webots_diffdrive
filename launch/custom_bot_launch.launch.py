# import os
# import launch
# from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from webots_ros2_driver.webots_launcher import WebotsLauncher
# from webots_ros2_driver.webots_controller import WebotsController
# from launch_ros.actions import Node


# def generate_launch_description():
    
#     package_dir = get_package_share_directory('webots_diffdrive')
#     robot_description_path = os.path.join(package_dir, 'resource', 'custom_robot.urdf')
#     webots = WebotsLauncher(
#         world=os.path.join(package_dir, 'worlds', 'custom_world.wbt')
#     )

#     robot_drivers = []
#     tf_broadcasters = []
#     world_to_robots = []
#     for i in range (1,3):
#         custom_robot_driver = WebotsController(
#             robot_name=f'robot{i}',
#             parameters=[
#                 {'robot_description': robot_description_path},
#             ],
#             namespace=f'robot{i}'

#         )
#         robot_drivers.append(custom_robot_driver)

#         lidar_tf_broadcaster = Node(
#             package='webots_diffdrive',
#             executable='lidar_tf_broadcaster',
#             output='screen',
#             namespace=f'robot{i}',
#             parameters=[{'robot_name': f'robot{i}'}],
#         )
#         tf_broadcasters.append(lidar_tf_broadcaster)

#         world_to_robot = Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name=f'world_to_robot{i}_broadcaster',
#             arguments=['0', '0', '0', '0', '0', '0', 'world', f'robot{i}/base_link']
#         )
#         world_to_robots.append(world_to_robot)


#     return LaunchDescription([
#         webots,
#         *robot_drivers,
#         *tf_broadcasters,
#         *world_to_robots,
#         launch.actions.RegisterEventHandler(
#             event_handler=launch.event_handlers.OnProcessExit(
#                 target_action=webots,
#                 on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
#             )
#         )
#     ])