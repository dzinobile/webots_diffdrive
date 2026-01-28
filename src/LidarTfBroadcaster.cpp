// #include "rclcpp/rclcpp.hpp"
// #include "tf2_ros/static_transform_broadcaster.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"

// class LidarTfBroadcaster : public rclcpp::Node {
// public:
//   LidarTfBroadcaster() : Node("lidar_tf_broadcaster") {
//     tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
//     // Publish static transforms for sensors
//     publish_lidar_transforms();
//   }

// private:
//   void publish_lidar_transforms() {
//     std::vector<geometry_msgs::msg::TransformStamped> transforms;

//     std::string ns = this->get_namespace();
//     if (ns == "/") ns = "";
//     else if (!ns.empty() && ns[0] == '/') ns = ns.substr(1);

//     std::string base_frame = ns.empty() ? "base_link" : ns + "/base_link";
    
//     // Lidar transform
//     geometry_msgs::msg::TransformStamped lidar_tf;
//     lidar_tf.header.stamp = this->get_clock()->now();
//     lidar_tf.header.frame_id = base_frame;
//     lidar_tf.child_frame_id = ns.empty() ? "lidar_frame" : ns + "/lidar_frame";
//     // Adjust these values based on your robot's sensor positions
//     lidar_tf.transform.translation.x = 0;
//     lidar_tf.transform.translation.y = 0;
//     lidar_tf.transform.translation.z = 0.05;
//     lidar_tf.transform.rotation.w = 1.0;
//     transforms.push_back(lidar_tf);
    
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