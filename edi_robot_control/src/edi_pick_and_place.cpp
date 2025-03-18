#include <edi_robot_control/edi_pick_and_place.h>
#include <csignal>

using namespace edi_pick_and_place;
using namespace manipulator_interface;
using namespace std::chrono_literals;

std::atomic_bool sigint_received(false);

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  EdiPickPlace application;

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(application.node_);

  std::thread executor_thread([&executor]() { executor.spin(); });
  executor_thread.detach();

  const rclcpp::Logger LOGGER = application.node_->get_logger();

  application.move_group_ptr = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(application.node_,"ur_manipulator"));
  
  application.gripper_action_client_ptr = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
                                            application.node_,"robotiq_gripper_controller/gripper_cmd"); //gripper

  application.tf_buffer_ptr = tfBufferPtr(new tf2_ros::Buffer(application.node_->get_clock()));
  application.tf_listener_ptr = tfListenerPtr(new tf2_ros::TransformListener(*application.tf_buffer_ptr));

  ManipulatorInterface manipulator(application.node_, application.move_group_ptr, application.gripper_action_client_ptr, 
                                   application.tf_buffer_ptr);

  rclcpp::Duration d = rclcpp::Duration::from_seconds(1.0);
  if(!application.gripper_action_client_ptr->wait_for_action_server(d.to_chrono<std::chrono::duration<double>>())) {
    RCLCPP_INFO(LOGGER, "waiting for gripper server to come up.");
  }

  application.moveit_visual_tools_->deleteAllMarkers();

  application.move_group_ptr->setStartState(*application.move_group_ptr->getCurrentState());

  RCLCPP_INFO(LOGGER, "PLANNER FRAME: %s", application.move_group_ptr->getPlanningFrame().c_str());
  
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::PoseStamped object_pose, pose_other;
  std_msgs::msg::String object;

  std::vector<geometry_msgs::msg::Pose> pick_poses;
  bool success;

  while(rclcpp::ok())// && !sigint_received)
  {

    application.moveit_visual_tools_->deleteAllMarkers();

    // manipulator.move_in_env();

    // application.moveit_visual_tools_->prompt("press 'Next' to open gripper");
    // manipulator.activate_gripper(false);

    // application.moveit_visual_tools_->prompt("press 'Next' to close gripper");
    // manipulator.activate_gripper(true);

    // NLP task planning demo START
    // application.moveit_visual_tools_->prompt("press 'Next' to action list");
    // std::vector<edi_robot_msgs::msg::ActionDesc> action_list = manipulator.process_actions();
    // manipulator.execute_actions(action_list);
    // NLP task planning demo END

  }

  rclcpp::shutdown();

  return 0;
}
