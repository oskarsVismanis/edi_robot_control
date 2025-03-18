// #ifndef MANIPULATOR_INTERFACE_H_
// #define MANIPULATOR_INTERFACE_H_

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include "control_msgs/action/gripper_command.hpp"
#include "edi_robot_msgs/srv/named_object_pose_stamped.hpp"
#include "edi_robot_msgs/srv/semantic_find_object.hpp"
#include "edi_robot_msgs/srv/get_grasp_pose.hpp"
#include "edi_robot_msgs/srv/get_action_list.hpp"
#include "edi_robot_msgs/srv/get_object_embedding.hpp"
#include "edi_robot_msgs/srv/get_command.hpp"

#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "edi_robot_msgs/msg/action_desc.hpp"
#include "edi_robot_msgs/msg/embedding.hpp"

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;

typedef boost::shared_ptr<tf2_ros::Buffer> tfBufferPtr;

typedef rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand> GoalHandleGripper;

namespace manipulator_interface
{
    struct SharedGripperData {
        control_msgs::action::GripperCommand::Goal gripper_goal;
        control_msgs::action::GripperCommand::Result::SharedPtr gripper_result;
        control_msgs::action::GripperCommand::Feedback::SharedPtr gripper_feedback;

        // Optional additions for extended state tracking
        double current_position = 0.0;
        double end_position = 0.0;

        // Synchronization primitives
        std::mutex m;
        std::condition_variable cv;
    };

    class ManipulatorInterface
    {
    public:
        ManipulatorInterface(const rclcpp::Node::SharedPtr& node, const MoveGroupPtr& move_group_ptr, 
                             const rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr& gripper_action_client_ptr, 
                             tfBufferPtr& tf_buffer_ptr); // Constructor

        ~ManipulatorInterface(); // Destructor

        rclcpp::Node::SharedPtr node_;
        moveit_visual_tools::MoveItVisualToolsPtr world_marker, robot_marker, zivid_camera_marker;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        bool debug = false;

        // HELPER FUNCTIONS
        /** \brief Generate a plan from a given trajectory 
           
            \param trajectory moveit_msgs::msg::RobotTrajectory message that includes trajectory to execute
            \return plan from input trajectory
        */
        moveit::planning_interface::MoveGroupInterface::Plan trajectory_generation(moveit_msgs::msg::RobotTrajectory &trajectory);

        /** \brief Helper function for create_pick_moves to generate a set of approach and retreat poses based on predefined distances
          
            \param target_tf The target transformation for the poses to be in. Usually from parent: world to parent: robot
            \return Vector of Poses for intended position and pre/post moving to it
         */
        std::vector<geometry_msgs::msg::Pose> create_manipulation_poses(double retreat_dis, double approach_dis, const tf2::Transform &target_tf);

        /** \brief Generate a set of poses for picking or placing an object 
           
            \param pose PoseStamped to reach for robot.
            \return Vector of Poses for intended position and pre/post moving to it
        */
        std::vector<geometry_msgs::msg::Pose> create_pick_moves(geometry_msgs::msg::Pose &pose);

        /** \brief Helper function to transform a pose to a different parent frame and visualize the process.

            \param target_frame the frame to which data should be transformed
            \param source_frame the frame where data originated
            \param pose transformable pose as geometry_msgs::msg::Pose
            \return Transformed pose as geometry_msgs::msg::PoseStamped
        */
        geometry_msgs::msg::Pose transform_pose(std::string target_frame, std::string source_frame, 
                                                geometry_msgs::msg::Pose pose, std::string frame_name = "Transformed");

        /** \brief function for adding a MoveIt collision object to the environment

            \param pose a Pose variable (usually a point on the object's surface)
                   to determine the objects dimensions
            \param frame_id std::string of the parent frame for the object
            \param coll_object moveit_msgs::msg::CollisionObject
            \return bool value of function success
        */
        bool add_collision_object(geometry_msgs::msg::Pose pose, std::string frame_id, moveit_msgs::msg::CollisionObject &coll_object);

        /** \brief function for attaching a collision object to the robot
           
            \param collision_object moveit_msgs::msg::CollisionObject of the object to attach
        */
        void attach_collision_object(moveit_msgs::msg::CollisionObject &collision_object);

        /** \brief function for dettaching a collision object from the robot
        */
        void detach_collision_object();
        
        /** \brief function for removing a collision object from the environment
        */
        void remove_collision_object();

        /** \brief function for getting a specific collision object's Z value
           
            \param object_id std::string of the object's name, to get
            \return a double value of the objects Z
        */
        double get_collision_object(std::string object_id = "object");

        /** \brief function to call voice-to-text (Whisper) service
           
            \return std::string of the received command
        */
        std::string get_command();

        // MANIPULATOR MOVEMENT FUNCTIONS
        /** \brief Move the robot to a named, predefined pose 
           
            \param pose_name predifined pose for robot to reach as string
            \return execution status as bool
        */
        bool predefined_pose(std::string pose_name);

        /** \brief Move the robot to a pose defined in cartesian space.  
            TODO: make overload that takes vector of poses so this can be used 
            to make robot move through multiple waypoints

            \param cartesian_pose cartesian pose for robot to reach as Pose
            \return execution status as bool
        */
        bool cartesian_goal(geometry_msgs::msg::Pose &cartesian_pose);

        // ACTION PRIMITIVES

        /** \brief action primitive for finding centroid of object.
          
            \param embedding Embedding of object name to find.
            \return object centroid as Pose.
         */
        geometry_msgs::msg::Pose find_in(edi_robot_msgs::msg::Embedding &embedding);
        
        /** \brief action primitive for getting grasp pose of object. 
          
            \return object grasp pose as Pose.
         */
        geometry_msgs::msg::Pose get_grasp_pose();

        /** \brief action primitive for moving the manipulator.
            Combines create_pick_moves and cartesian_goal
          
            \param pose_stamped PoseStamped to reach for robot.
            \return execution status as bool
         */
        bool move_manipulator(geometry_msgs::msg::PoseStamped &pose_stamped);

        /** \brief action primitive for moving the manipulator.
            Uses cartesian_goal
          
            \param pose Pose to reach for robot.
            \return execution status as bool
         */
        bool move_manipulator(geometry_msgs::msg::Pose &pose);

        /** \brief action primitive for moving the manipulator
          
            \param pose_name String of predefined pose name.
            \return execution status as bool
         */
        bool move_manipulator(std::string pose_name);

        // void move(edi_robot_msgs::msg::ActionDesc &action_desc);

        /** \brief Activate (close/open) gripper 
            
            \param do_grasp true to close, false to open
            \return execution status as bool
         */
        moveit::core::MoveItErrorCode activate_gripper(bool do_grasp);
        
        // ACTIONS

        /** \brief action for picking up an object
            Combines the action primitives find_in_env, get_grasp, move_manipulator, set_gripper
          
            \param pick_pose Pose of picking position for robot
            \return execution status as bool
        */
        bool pick_up(geometry_msgs::msg::Pose &pick_pose);
        // bool pick_up(std_msgs::msg::String &object);

        /** \brief action for placing an object
            Combines the action primitives move_manipulator, set_gripper

            \param place_pose Pose of placing position for robot
            \return execution status as bool
        */
        bool put_down(geometry_msgs::msg::Pose &place_pose);

        /** \brief helper function for moving the robot over the workspace, to create
            a map with a camera.
        */
        void map_env();

        /** \brief function to check the safety of the position
            Checks if the poses in the list meet the safety requirements for depth and gripper rotation 
            \param poses a std::vector<geometry_msgs::msg::Pose> of poses
            \param added_height a double value of added height, which can be set for safety or
                   taken from the collision object, so the real object wouldn't collide with
                   the environment
            \param is_place a bool value for determining if the action is pick or place
            TODO: make the robot rotate the gripper less in certain positions
        */
        void check_pose(std::vector<geometry_msgs::msg::Pose>& pick_poses, double added_height = 0.0, bool is_place = false);
        
    private:

        MoveGroupPtr move_group_ptr_; // Member variable to store MoveGroupPtr
        rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_ptr_;
        tfBufferPtr& tf_buffer_ptr_;

        bool success, exec_success;
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        SharedGripperData shared_gripper_data_;
    };

} // namespace manipulator_interface
// #endif /* MANIPULATOR_INTERFACE_H_ */