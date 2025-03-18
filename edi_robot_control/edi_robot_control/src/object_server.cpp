#include "rclcpp/rclcpp.hpp"
// #include "example_interfaces/srv/add_two_ints.hpp"
// #include "edi_robot_msgs/srv/named_object_pose.hpp"
#include "edi_robot_msgs/srv/named_object_pose_stamped.hpp"

#include <memory>

// void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
//           std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
// {
//   response->sum = request->a + request->b;
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
//                 request->a, request->b);
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
// }

void object_pose_list(const std::shared_ptr<edi_robot_msgs::srv::NamedObjectPoseStamped::Request> request,
    std::shared_ptr<edi_robot_msgs::srv::NamedObjectPoseStamped::Response>      response)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::PoseStamped object_pose;
  std_msgs::msg::String object_name = request->object;

  object_pose.header.frame_id = "zivid_optical_frame";

  if(object_name.data == "new_object")
  {
    object_pose.pose.position.x = -0.09425716;
    object_pose.pose.position.y = -0.0702383;
    object_pose.pose.position.z = 1.3873167;
    object_pose.pose.orientation.x = 0.701;
    object_pose.pose.orientation.y = 0.701;
    object_pose.pose.orientation.z = -0.092;
    object_pose.pose.orientation.w = 0.092;
  }

  if(object_name.data == "stapler")
  {
    object_pose.pose.position.x = 0.0100081772;
    object_pose.pose.position.y = -0.0910612267;
    object_pose.pose.position.z = 1.39935637;
    object_pose.pose.orientation.x = 0.779885589;
    object_pose.pose.orientation.y = -0.625922095;
    object_pose.pose.orientation.z = -0.0;
    object_pose.pose.orientation.w = 0.0;
  }

  if(object_name.data == "scissors")
  {
    object_pose.pose.position.x = -0.213069485;
    object_pose.pose.position.y = 0.0928231632;
    object_pose.pose.position.z = 1.36870670;
    object_pose.pose.orientation.x = 0.970373680;
    object_pose.pose.orientation.y = -0.241609024;
    object_pose.pose.orientation.z = -0.0;
    object_pose.pose.orientation.w = 0.0;
  }

  if(object_name.data == "screwdriver")
  {
    object_pose.pose.position.x = -0.28;
    object_pose.pose.position.y = 0.2;
    object_pose.pose.position.z = 1.3;
    object_pose.pose.orientation.x = 0.701;
    object_pose.pose.orientation.y = 0.701;
    object_pose.pose.orientation.z = -0.092;
    object_pose.pose.orientation.w = 0.092;
  }

  if(object_name.data == "bottle")
  {
    object_pose.pose.position.x = -0.28;
    object_pose.pose.position.y = -0.11;
    object_pose.pose.position.z = 1.4;
    object_pose.pose.orientation.x = 0.701;
    object_pose.pose.orientation.y = 0.701;
    object_pose.pose.orientation.z = -0.092;
    object_pose.pose.orientation.w = 0.092;
  }

  if(object_name.data == "can")
  {
    object_pose.pose.position.x = 0.17;
    object_pose.pose.position.y = -0.10;
    object_pose.pose.position.z = 1.4;
    object_pose.pose.orientation.x = 0.701;
    object_pose.pose.orientation.y = 0.701;
    object_pose.pose.orientation.z = -0.092;
    object_pose.pose.orientation.w = 0.092;
  }

  response->pose_stamped = object_pose;
  // response->pose_stamped.header.frame_id = "zivid_optical_frame";

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nobject: %s", request->object.data.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response\nx: [%f]\ny: [%f]\nz: [%f]\nqx: [%f]\nqy: [%f]\nqz: [%f]\nqw: [%f]", 
            response->pose_stamped.pose.position.x, response->pose_stamped.pose.position.y, response->pose_stamped.pose.position.z,
            response->pose_stamped.pose.orientation.x, response->pose_stamped.pose.orientation.y, 
            response->pose_stamped.pose.orientation.z, response->pose_stamped.pose.orientation.w);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("object_server");

  // rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
  //   node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  rclcpp::Service<edi_robot_msgs::srv::NamedObjectPoseStamped>::SharedPtr service =
  node->create_service<edi_robot_msgs::srv::NamedObjectPoseStamped>("object_poses", &object_pose_list);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to send object poses.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}