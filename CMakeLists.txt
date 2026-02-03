cmake_minimum_required(VERSION 3.5)
project(webots_diffdrive)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

# Export the plugin configuration file
# pluginlib_export_plugin_description_file(webots_ros2_driver my_robot_driver.xml)
pluginlib_export_plugin_description_file(webots_ros2_driver custom_robot_driver.xml)

# MyRobotDriver library
add_library(
  ${PROJECT_NAME}
  SHARED
  # src/MyRobotDriver.cpp
  src/CustomRobotDriver.cpp
)


target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2
)



# add_executable(sensor_tf_broadcaster src/SensorTfBroadcaster.cpp)
# ament_target_dependencies(
#   sensor_tf_broadcaster
#   rclcpp
#   tf2_ros
#   tf2
#   geometry_msgs
#   sensor_msgs
# )

# add_executable(lidar_tf_broadcaster src/LidarTfBroadcaster.cpp)
# ament_target_dependencies(
#   lidar_tf_broadcaster
#   rclcpp
#   tf2
#   tf2_ros
#   geometry_msgs
#   sensor_msgs
# )

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS
#         sensor_tf_broadcaster
#         DESTINATION lib/${PROJECT_NAME})

# install(TARGETS
#         lidar_tf_broadcaster
#         DESTINATION lib/${PROJECT_NAME})
# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  )
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  pluginlib
  webots_ros2_driver
)
ament_package()