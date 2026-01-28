// #include "rclcpp/rclcpp.hpp"
// #include "tf2_ros/static_transform_broadcaster.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"

// class LidarTfBroadcaster : public rclcpp::Node {
// public:
//   LidarTfBroadcaster() : Node("sensor_tf_broadcaster") {
//     tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
//     // Publish static transforms for sensors
//     publish_sensor_transforms();
//   }

// private:
//   void publish_sensor_transforms() {
//     std::vector<geometry_msgs::msg::TransformStamped> transforms;

//     std::string ns = this->get_namespace();
//     if (ns == "/") ns = "";
//     else if (!ns.empty() && ns[0] == '/') ns = ns.substr(1);

//     std::string base_frame = ns.empty() ? "base_link" : ns + "/base_link";
    
//     // Left sensor transform
//     geometry_msgs::msg::TransformStamped left_sensor_tf;
//     left_sensor_tf.header.stamp = this->get_clock()->now();
//     left_sensor_tf.header.frame_id = "base_link";
//     left_sensor_tf.child_frame_id = "left_sensor_frame";
//     // Adjust these values based on your robot's sensor positions
//     left_sensor_tf.transform.translation.x = 0.05;
//     left_sensor_tf.transform.translation.y = 0.02;
//     left_sensor_tf.transform.translation.z = 0.01;
//     left_sensor_tf.transform.rotation.w = 1.0;
//     transforms.push_back(left_sensor_tf);
    
//     // Right sensor transform
//     geometry_msgs::msg::TransformStamped right_sensor_tf;
//     right_sensor_tf.header.stamp = this->get_clock()->now();
//     right_sensor_tf.header.frame_id = "base_link";
//     right_sensor_tf.child_frame_id = "right_sensor_frame";
//     // Adjust these values based on your robot's sensor positions
//     right_sensor_tf.transform.translation.x = 0.05;
//     right_sensor_tf.transform.translation.y = -0.02;
//     right_sensor_tf.transform.translation.z = 0.01;
//     right_sensor_tf.transform.rotation.w = 1.0;
//     transforms.push_back(right_sensor_tf);
    
//     // Broadcast all transforms
//     tf_static_broadcaster_->sendTransform(transforms);
//     RCLCPP_INFO(this->get_logger(), "Published sensor transforms");
//   }

//   std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<LidarTfBroadcaster>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }