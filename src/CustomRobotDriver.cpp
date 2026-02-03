#include "webots_diffdrive/CustomRobotDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/lidar.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.05
#define WHEEL_RADIUS 0.025

namespace my_robot_driver {
void MyRobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  right_motor = wb_robot_get_device("right wheel motor");
  left_motor = wb_robot_get_device("left wheel motor");
  test_motor = wb_robot_get_device("test motor");
  
  WbDeviceTag lidar = wb_robot_get_device("ld0");
  int timestep = wb_robot_get_basic_time_step();
  wb_lidar_enable(lidar, timestep);
  wb_lidar_enable_point_cloud(lidar);

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);

  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);

  wb_motor_set_position(test_motor, INFINITY);
  wb_motor_set_velocity(test_motor, 0.0);

  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&MyRobotDriver::cmdVelCallback, this, std::placeholders::_1));
  test_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/test_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&MyRobotDriver::testVelCallback, this, std::placeholders::_1));
  
}

void MyRobotDriver::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_msg.linear = msg->linear;
  cmd_vel_msg.angular = msg->angular;
}

void MyRobotDriver::testVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  test_vel_msg.linear = msg->linear;
  test_vel_msg.angular = msg->angular;
  }


void MyRobotDriver::step() {
  auto forward_speed = cmd_vel_msg.linear.x;
  auto angular_speed = cmd_vel_msg.angular.z;
  auto test_speed = test_vel_msg.angular.z;

  auto command_motor_left =
      (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto command_motor_right =
      (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto command_motor_test = test_speed;

  wb_motor_set_velocity(left_motor, command_motor_left);
  wb_motor_set_velocity(right_motor, command_motor_right);
  wb_motor_set_velocity(test_motor, command_motor_test);
}
} // namespace my_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_driver::MyRobotDriver,
                       webots_ros2_driver::PluginInterface)