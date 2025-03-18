#include "rclcpp/rclcpp.hpp"
// #include "edi_robot_msgs/srv/named_object_pose.hpp"
#include "edi_robot_msgs/srv/named_object_pose_stamped.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  if (argc != 2) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: object_server_client X (string)");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("object_server_client");
  rclcpp::Client<edi_robot_msgs::srv::NamedObjectPoseStamped>::SharedPtr client =
    node->create_client<edi_robot_msgs::srv::NamedObjectPoseStamped>("object_poses");

  auto request = std::make_shared<edi_robot_msgs::srv::NamedObjectPoseStamped::Request>();
  request->object.data = argv[1];

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  geometry_msgs::msg::PoseStamped object_pose;

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    object_pose = result.get()->pose_stamped;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "X COORDINATE: %f", object_pose.pose.position.x);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Y COORDINATE: %f", object_pose.pose.position.y);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Z COORDINATE: %f", object_pose.pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "QX COORDINATE: %f", object_pose.pose.orientation.x);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "QY COORDINATE: %f", object_pose.pose.orientation.y);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "QZ COORDINATE: %f", object_pose.pose.orientation.z);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "QW COORDINATE: %f", object_pose.pose.orientation.w);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FRAME ID: %s", object_pose.header.frame_id.c_str());

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose received");

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}