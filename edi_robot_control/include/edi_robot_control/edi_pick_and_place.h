#ifndef EDI_PICK_AND_PLACE_H_
#define EDI_PICK_AND_PLACE_H_

#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// #include "imoco_msgs/srv/get_object_pose.hpp"
#include "edi_robot_msgs/srv/named_object_pose_stamped.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include <manipulator_interface/manipulator_interface.h>
// #include <primitives_interface/primitives_interface.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;
typedef boost::shared_ptr<tf2_ros::Buffer> tfBufferPtr;

// gripper
// typedef rclcpp_action::Client<control_msgs::action::GripperCommand> GripperActionClient;
// typedef std::shared_ptr<GripperActionClient> GripperActionClientPtr; // change this bullshit later
typedef rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand> GoalHandleGripper;

// tests
// typedef boost::shared_ptr<manipulator_interface::ManipulatorInterface> ManipulatorInterfacePtr;

namespace edi_pick_and_place
{
	class EdiPickPlace
	{
	public:
		EdiPickPlace()
		{
			node_options.use_intra_process_comms(false);
			node_ = std::make_shared<rclcpp::Node>("edi_pick_and_place_node", node_options);

			moveit_visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(node_, "base_link", "/moveit_visual_markers"));
			moveit_visual_tools_->loadPlanningSceneMonitor();
			moveit_visual_tools_->loadMarkerPub(true);
			moveit_visual_tools_->waitForMarkerSub();
			moveit_visual_tools_->loadRobotStatePub("display_robot_state");
			moveit_visual_tools_->setManualSceneUpdating();
			moveit_visual_tools_->loadRemoteControl();
			moveit_visual_tools_->deleteAllMarkers();

			// zivid_camera_marker.reset(new moveit_visual_tools::MoveItVisualTools(node_, "zivid_optical_frame", "/zivid_camera_markers"));
			// zivid_camera_marker->loadPlanningSceneMonitor();
			// zivid_camera_marker->loadMarkerPub(true);
			// zivid_camera_marker->waitForMarkerSub();
			// zivid_camera_marker->loadRobotStatePub("display_robot_state");
			// zivid_camera_marker->setManualSceneUpdating();
			// zivid_camera_marker->loadRemoteControl();
			// zivid_camera_marker->deleteAllMarkers();

			// world_marker.reset(new moveit_visual_tools::MoveItVisualTools(node_, "world", "/world_marker"));
			// world_marker->loadPlanningSceneMonitor();
			// world_marker->loadMarkerPub(true);
			// world_marker->waitForMarkerSub();
			// world_marker->loadRobotStatePub("display_robot_state");
			// world_marker->setManualSceneUpdating();
			// world_marker->loadRemoteControl();
			// world_marker->deleteAllMarkers();
		}
		
		rclcpp::NodeOptions node_options;
		rclcpp::Node::SharedPtr node_;
		moveit_visual_tools::MoveItVisualToolsPtr moveit_visual_tools_;//, zivid_camera_marker, world_marker;

		// rclcpp::Client<imoco_msgs::srv::GetObjectPose>::SharedPtr object_detection_service;
		// rclcpp::Client<edi_robot_msgs::srv::NamedObjectPoseStamped>::SharedPtr named_object_service;

		// imoco_msgs::srv::GetObjectPose srv;

		//tests
		// ManipulatorInterfacePtr manipulator_interface_ptr;

		MoveGroupPtr move_group_ptr;
		rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_ptr;
		
		// moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		// const moveit::core::JointModelGroup* joint_model_group;
		// const moveit::core::LinkModel* end_effector;
		// bool success, exec_success;
		// moveit::planning_interface::MoveGroupInterface grp;
		tfBufferPtr tf_buffer_ptr;
		tfListenerPtr tf_listener_ptr;
		// std::vector<geometry_msgs::msg::Pose> waypoints;
		

		// geometry_msgs::msg::Pose detect_object();
		// geometry_msgs::msg::PoseStamped get_named_object_pose(std_msgs::msg::String &object);
		
	};
}


#endif /* EDI_PICK_AND_PLACE_H_ */