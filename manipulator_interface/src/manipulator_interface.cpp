// #include "manipulator_interface.h"
#include <manipulator_interface/manipulator_interface.h>

// using namespace manipulator_interface;
using namespace std::chrono_literals;

namespace manipulator_interface
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("manipulator_interface");

ManipulatorInterface::ManipulatorInterface(const rclcpp::Node::SharedPtr& node, const MoveGroupPtr& move_group_ptr, 
                                           const rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr& gripper_action_client_ptr,
										   tfBufferPtr& tf_buffer_ptr)
    : node_(node), move_group_ptr_(move_group_ptr), 
      gripper_action_client_ptr_(gripper_action_client_ptr),
	  tf_buffer_ptr_(tf_buffer_ptr)
{
    // Constructor implementation  
    world_marker.reset(new moveit_visual_tools::MoveItVisualTools(node_, "world", "/world_marker"));
    world_marker->loadPlanningSceneMonitor();
    world_marker->loadMarkerPub(true);
    world_marker->waitForMarkerSub();
    world_marker->loadRobotStatePub("display_robot_state");
    world_marker->setManualSceneUpdating();
    world_marker->loadRemoteControl();
    world_marker->deleteAllMarkers();

	robot_marker.reset(new moveit_visual_tools::MoveItVisualTools(node_, "base_link", "/robot_marker"));
    robot_marker->loadPlanningSceneMonitor();
    robot_marker->loadMarkerPub(true);
    robot_marker->waitForMarkerSub();
    robot_marker->loadRobotStatePub("display_robot_state");
    robot_marker->setManualSceneUpdating();
    robot_marker->loadRemoteControl();
    robot_marker->deleteAllMarkers();

	zivid_camera_marker.reset(new moveit_visual_tools::MoveItVisualTools(node_, "zivid_optical_frame", "/zivid_camera_markers"));
    zivid_camera_marker->loadPlanningSceneMonitor();
    zivid_camera_marker->loadMarkerPub(true);
    zivid_camera_marker->waitForMarkerSub();
    zivid_camera_marker->loadRobotStatePub("display_robot_state");
    zivid_camera_marker->setManualSceneUpdating();
    zivid_camera_marker->loadRemoteControl();
    zivid_camera_marker->deleteAllMarkers();
}

// const rclcpp::Logger LOGGER = rclcpp::get_logger("manipulator_interface");

ManipulatorInterface::~ManipulatorInterface()
{
    // Destructor implementation
}

moveit::planning_interface::MoveGroupInterface::Plan ManipulatorInterface::trajectory_generation(moveit_msgs::msg::RobotTrajectory &trajectory)
{
	int num_iterations = 10;  // Set a maximum number of iterations
	double best_path_length = std::numeric_limits<double>::max();  // Initialize with a large value
	moveit_msgs::msg::RobotTrajectory best_trajectory;  // Store the best trajectory

	const double desired_waypoints_threshold = 50.0;  // Stop search if fewer than this many waypoints are found
	bool found_desired_trajectory = false;  // Flag to track if we've found a valid trajectory with less than 50 waypoints

	for (int i = 0; i < num_iterations; ++i) {
		RCLCPP_INFO(LOGGER, "Planning attempt #%d", i + 1);

		// Choose the trajectory time parameterization method
		// trajectory_processing::IterativeParabolicTimeParameterization iptp;
		trajectory_processing::TimeOptimalTrajectoryGeneration iptp;

		// Create a RobotTrajectory object
		robot_trajectory::RobotTrajectory rt(move_group_ptr_->getRobotModel(), move_group_ptr_->getName());
		rt.setRobotTrajectoryMsg(*move_group_ptr_->getCurrentState(), trajectory);

		// Generate timestamps for the trajectory
		bool success = iptp.computeTimeStamps(rt);

		// If the planning is successful, proceed
		if (success) {
			// Convert the robot trajectory back to message form
			rt.getRobotTrajectoryMsg(trajectory);

			// Calculate the path length
			double path_length = rt.getWayPointCount();  // Get the number of waypoints in the trajectory
			RCLCPP_INFO(LOGGER, "Path length in iteration #%d: %f", i + 1, path_length);

			// Check if the current trajectory has fewer waypoints than the threshold
			if (path_length < desired_waypoints_threshold) {
				RCLCPP_INFO(LOGGER, "Found trajectory with less than %f waypoints after %d iterations", desired_waypoints_threshold, i + 1);
				found_desired_trajectory = true;
				best_path_length = path_length;
				best_trajectory = trajectory;  // Store this trajectory
				break;  // Exit the loop as we found a satisfactory trajectory
			}

			// Also store the trajectory if it's better than the previous best
			if (path_length < best_path_length) {
				best_path_length = path_length;
				best_trajectory = trajectory;  // Store this as the best trajectory
			}
		} else {
			RCLCPP_WARN(LOGGER, "Planning attempt #%d failed", i + 1);
		}
	}

	// After all iterations, or if the desired trajectory is found, use the best trajectory
	if (found_desired_trajectory) {
		RCLCPP_INFO(LOGGER, "Successfully found a path with fewer than %f waypoints.", desired_waypoints_threshold);
	} else {
		RCLCPP_WARN(LOGGER, "Did not find a path with fewer than %f waypoints, using the best found trajectory.", desired_waypoints_threshold);
	}

	plan.trajectory_ = best_trajectory;

	RCLCPP_INFO(LOGGER, "Best path length found: %f", best_path_length);

	return plan;
}

std::vector<geometry_msgs::msg::Pose> ManipulatorInterface::create_manipulation_poses(
														double retreat_dis, double approach_dis, const tf2::Transform &target_tf)
{
  geometry_msgs::msg::Pose start_pose, target_pose, end_pose;
  std::vector<geometry_msgs::msg::Pose> poses;

  // creating start pose by applying a translation along +z by approach distance
  tf2::Transform start_tf = target_tf * tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, -approach_dis));
  tf2::toMsg(start_tf, start_pose);

  // Convert the target pose
  tf2::toMsg(target_tf, target_pose);

  // Calculate the end pose by applying a translation along -z by retreat distance
  tf2::Transform end_tf = target_tf * tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, -retreat_dis));
  tf2::toMsg(end_tf, end_pose);

  poses.push_back(start_pose);
  poses.push_back(target_pose);
  poses.push_back(end_pose);

  return poses;
}

std::vector<geometry_msgs::msg::Pose> ManipulatorInterface::create_pick_moves(geometry_msgs::msg::Pose &pose)
{
	// variables
	tf2::Transform world_to_tcp_tf;
	tf2::Transform world_to_object_tf;
	std::vector<geometry_msgs::msg::Pose> wrist_poses;

	RCLCPP_INFO(LOGGER, "object pose %f %f %f", pose.position.x, pose.position.y, pose.position.z);

	tf2::fromMsg(pose, world_to_object_tf);
	tf2::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
    world_to_tcp_tf.setOrigin(position);
	// std_msgs::msg::String origin = world_to_tcp_tf.getOrigin();
	// RCLCPP_INFO(node_->get_logger(), "origin %s", origin.c_str());

 	world_to_tcp_tf.setRotation(world_to_object_tf.getRotation() * tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI));

	double RETREAT_DISTANCE  = 0.04f;
  	double APPROACH_DISTANCE = 0.04f;

	wrist_poses = create_manipulation_poses(RETREAT_DISTANCE, APPROACH_DISTANCE, world_to_tcp_tf);

	world_marker->deleteAllMarkers();
	world_marker->publishAxisLabeled(wrist_poses[0], "world_pre");
	world_marker->publishAxisLabeled(wrist_poses[1], "world_grasp");
	world_marker->trigger();

	RCLCPP_INFO(LOGGER, "pick pose 1 %f %f %f", wrist_poses[0].position.x, wrist_poses[0].position.y, wrist_poses[0].position.z);
	RCLCPP_INFO(LOGGER, "pick pose 2 %f %f %f", wrist_poses[1].position.x, wrist_poses[1].position.y, wrist_poses[1].position.z);
	RCLCPP_INFO(LOGGER, "pick pose 3 %f %f %f", wrist_poses[2].position.x, wrist_poses[2].position.y, wrist_poses[2].position.z);

	return wrist_poses;
}

geometry_msgs::msg::Pose ManipulatorInterface::transform_pose(std::string target_frame, std::string source_frame, 
															  geometry_msgs::msg::Pose pose, std::string frame_name)
{
	geometry_msgs::msg::PoseStamped pose_stamped, pose_stamped_transformed;
	geometry_msgs::msg::TransformStamped transform;

	// bool debug = true;

	// target_frame = "world";
	// source_frame = "zivid_optical_frame";

	pose_stamped.header.frame_id = source_frame;
	pose_stamped.pose = pose;

	RCLCPP_ERROR(LOGGER, "[transform_pose] Received pose z value: %f ", pose.position.z);

	if(pose.orientation.x == 0 && pose.orientation.y == 0 && pose.orientation.z == 0 && pose.orientation.w == 0)
	{
		RCLCPP_INFO(LOGGER, "Received Pose without orientation");
		pose.orientation.x = 0.701;
        pose.orientation.y = 0.701;
        pose.orientation.z = -0.092;
        pose.orientation.w = 0.092;
	} else {
		RCLCPP_INFO(LOGGER, "Received Pose with orientation");
	}

	RCLCPP_INFO(LOGGER, "Pose header before transform: %s", pose_stamped.header.frame_id.c_str());

	if(source_frame == "zivid_optical_frame"){
		zivid_camera_marker->deleteAllMarkers();
		zivid_camera_marker->trigger();
		zivid_camera_marker->publishAxisLabeled(pose_stamped.pose, "PreTransform");
		zivid_camera_marker->trigger();
	} else if(source_frame == "base_link"){
		robot_marker->deleteAllMarkers();
		robot_marker->trigger();
		robot_marker->publishAxisLabeled(pose_stamped.pose, "PreTransform");
		robot_marker->trigger();
	}

	transform = tf_buffer_ptr_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

	tf2::doTransform(pose_stamped, pose_stamped_transformed, transform);

	RCLCPP_INFO(LOGGER, "Pose header after transform: %s", pose_stamped_transformed.header.frame_id.c_str());
    world_marker->publishAxisLabeled(pose_stamped_transformed.pose, frame_name);
	world_marker->trigger();

    // Testing rotations START
    tf2::Quaternion quaternion(pose_stamped_transformed.pose.orientation.x, 
                               pose_stamped_transformed.pose.orientation.y, 
                               pose_stamped_transformed.pose.orientation.z, 
                               pose_stamped_transformed.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    quaternion.setRPY(0, 0, yaw);

    pose_stamped_transformed.pose.orientation.x = quaternion[0];
    pose_stamped_transformed.pose.orientation.y = quaternion[1]; 
    pose_stamped_transformed.pose.orientation.z = quaternion[2]; 
    pose_stamped_transformed.pose.orientation.w = quaternion[3]; 
    // Testing rotations END

	RCLCPP_ERROR(LOGGER, "[transform_pose] Transformed pose z value: %f ", pose_stamped_transformed.pose.position.z);

	if(debug){
		world_marker->prompt("press 'Next' to create perpendicular-to-plane pose");
		RCLCPP_INFO(LOGGER, "Pose header after rotation: %s", pose_stamped_transformed.header.frame_id.c_str());
	}
	world_marker->deleteAllMarkers();
    world_marker->trigger();
    world_marker->publishAxisLabeled(pose_stamped_transformed.pose, frame_name);
	world_marker->trigger();

	return pose_stamped_transformed.pose;
}

bool ManipulatorInterface::predefined_pose(std::string pose_name)
{
	move_group_ptr_->setStartState(*move_group_ptr_->getCurrentState());
	move_group_ptr_->setNamedTarget(pose_name);

	success = move_group_ptr_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;

	if(success){
	    plan = trajectory_generation(plan.trajectory_);  
	    exec_success = move_group_ptr_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
	    if(exec_success){
	    	return exec_success;
	    } else {
            RCLCPP_INFO(LOGGER, "Plan is found for pose: %s but execution failed!", pose_name.c_str());
	    	return exec_success;
	    }
	} else {
        RCLCPP_INFO(LOGGER, "Failed to plan to: %s", pose_name.c_str());
		return success;
	}
}

bool ManipulatorInterface::cartesian_goal(geometry_msgs::msg::Pose &cartesian_pose)
{
	RCLCPP_INFO(LOGGER, "NEW VERSION OF cartesian_goal");
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;
    double fraction;
    std::vector<geometry_msgs::msg::Pose> waypoints;

    const moveit::core::JointModelGroup* joint_model_group = move_group_ptr_->getCurrentState()->getJointModelGroup(move_group_ptr_->getName());
    const moveit::core::LinkModel* end_effector = move_group_ptr_->getCurrentState()->getLinkModel(move_group_ptr_->getEndEffectorLink());

    waypoints.clear();
    move_group_ptr_->setStartState(*move_group_ptr_->getCurrentState());
    waypoints.push_back(move_group_ptr_->getCurrentPose(move_group_ptr_->getEndEffectorLink().c_str()).pose);
    waypoints.push_back(cartesian_pose);

    //TRAJECTORY GENERATION START
    int num_iterations = 10;  // Set a maximum number of iterations
    double best_path_length = std::numeric_limits<double>::max();  // Initialize with a large value
    moveit_msgs::msg::RobotTrajectory best_trajectory;  // Store the best trajectory
    const double desired_waypoints_threshold = 50.0;  // Stop search if fewer than this many waypoints are found

    bool found_desired_trajectory = false;  // Flag to track if we've found a valid trajectory

    for (int i = 0; i < num_iterations; ++i) {
        RCLCPP_INFO(LOGGER, "Planning attempt #%d", i + 1);

        // Compute Cartesian path
        fraction = move_group_ptr_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        // If full path is computed, proceed with time parameterization
        if (fraction == 1.0) {
            trajectory_processing::TimeOptimalTrajectoryGeneration iptp;
            robot_trajectory::RobotTrajectory rt(move_group_ptr_->getRobotModel(), move_group_ptr_->getName());
            rt.setRobotTrajectoryMsg(*move_group_ptr_->getCurrentState(), trajectory);

            // Generate timestamps for the trajectory
            bool plan_success = iptp.computeTimeStamps(rt);

            if (plan_success) {
                rt.getRobotTrajectoryMsg(trajectory);

                // Calculate path length (number of waypoints)
                double path_length = rt.getWayPointCount();
                RCLCPP_INFO(LOGGER, "Path length in iteration #%d: %f", i + 1, path_length);

                // Check if the path length meets the desired threshold
                if (path_length < desired_waypoints_threshold) {
                    RCLCPP_INFO(LOGGER, "Found trajectory with fewer than %f waypoints after %d iterations", desired_waypoints_threshold, i + 1);
                    best_trajectory = trajectory;
                    found_desired_trajectory = true;
                    break;  // Exit loop if satisfactory trajectory is found
                }

                // Store the trajectory if it's better than the previous best
                if (path_length < best_path_length) {
                    best_path_length = path_length;
                    best_trajectory = trajectory;
                }
            } else {
                RCLCPP_WARN(LOGGER, "Planning attempt #%d failed (time parameterization)", i + 1);
            }
        } else {
            RCLCPP_WARN(LOGGER, "Planning attempt #%d failed (incomplete Cartesian path)", i + 1);
        }
    }

    // After all iterations, check if we found a valid trajectory
    if (found_desired_trajectory) {
        plan.trajectory_ = best_trajectory;
        RCLCPP_INFO(LOGGER, "Successfully found a path with fewer than %f waypoints. Best path length: %f", desired_waypoints_threshold, best_path_length);
    } else {
        RCLCPP_WARN(LOGGER, "Did not find a path with fewer than %f waypoints, failing the plan.", desired_waypoints_threshold);
		world_marker->prompt("Press 'Next' to continue");
        return false;  // Fail if no valid trajectory is found
    }

    // Publish the trajectory for visualization
    world_marker->publishTrajectoryLine(plan.trajectory_, end_effector, joint_model_group);
    world_marker->trigger();

    // Execute the plan if a valid trajectory was found
    world_marker->prompt("Press 'Next' to execute the cartesian plan");
    RCLCPP_INFO(LOGGER, "Executing cartesian plan");
    bool exec_success = move_group_ptr_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    world_marker->deleteAllMarkers();
    world_marker->trigger();

    return exec_success;  // Return success/failure based on the execution result
}

// ACTION PRIMITIVES
moveit::core::MoveItErrorCode ManipulatorInterface::activate_gripper(bool do_grasp){

	// Log the start of primitive [for debugging]
	RCLCPP_WARN(LOGGER, "Entering new activate_gripper function");

    // Reset shared gripper result and set maximum effort
    {
        std::unique_lock<std::mutex> lock(shared_gripper_data_.m);
        shared_gripper_data_.gripper_result = nullptr;
        shared_gripper_data_.gripper_goal.command.max_effort = 77.0;
    }

	{
		// Lock the mutex before accessing shared_gripper_data_
		std::unique_lock<std::mutex> lock(shared_gripper_data_.m);

		// Update the current position using the value from MoveGroup
		shared_gripper_data_.current_position = 
			move_group_ptr_->getCurrentState()->getVariablePosition("finger_joint");

		RCLCPP_INFO(LOGGER, "Position of 'finger_joint': %f", shared_gripper_data_.current_position);
	}

    // Set gripper position based on `do_grasp`
    if (do_grasp) {
        shared_gripper_data_.gripper_goal.command.position = 0.695;
        RCLCPP_INFO(LOGGER, "Gripper goal set to grasp with position: %f", 
                    shared_gripper_data_.gripper_goal.command.position);
    } else {
        shared_gripper_data_.gripper_goal.command.position = 0.15;
        RCLCPP_INFO(LOGGER, "Gripper goal set to release with position: %f", 
                    shared_gripper_data_.gripper_goal.command.position);
    }

    // Log the gripper command being sent
    RCLCPP_INFO(LOGGER, "Sending gripper command with position: %f and max effort: %f", 
                shared_gripper_data_.gripper_goal.command.position, 
                shared_gripper_data_.gripper_goal.command.max_effort);
    
    // Define goal options for sending the action command
    auto goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();

    // Goal response callback
    goal_options.goal_response_callback = [this](const GoalHandleGripper::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_INFO(LOGGER, "Gripper goal request rejected!");
        } else {
            RCLCPP_INFO(LOGGER, "Gripper goal request accepted");
        }
    };

	// Result callback
    goal_options.result_callback = [this](const GoalHandleGripper::WrappedResult& result) {
        std::unique_lock<std::mutex> lock(shared_gripper_data_.m);
        shared_gripper_data_.gripper_result = result.result;

        if (!shared_gripper_data_.gripper_result->reached_goal) {
            RCLCPP_ERROR(LOGGER, "Gripper goal execution failed!");
            shared_gripper_data_.end_position = result.result->position;
        } else {
            RCLCPP_INFO(LOGGER, "Gripper goal executed successfully");
            RCLCPP_INFO(LOGGER, "Gripper position: %f", result.result->position);
            RCLCPP_INFO(LOGGER, "Gripper effort: %f", result.result->effort);
            shared_gripper_data_.end_position = result.result->position;
        }
        shared_gripper_data_.cv.notify_all(); // Notify waiting threads
    };

    // Feedback callback
    goal_options.feedback_callback = [this](const GoalHandleGripper::SharedPtr&, 
                                            const std::shared_ptr<const control_msgs::action::GripperCommand::Feedback>& feedback) {
        RCLCPP_WARN(LOGGER, "Feedback callback triggered");
		std::unique_lock<std::mutex> lock(shared_gripper_data_.m);
        shared_gripper_data_.current_position = feedback->position;
        RCLCPP_INFO(LOGGER, "Gripper current position feedback: %f", feedback->position);
        RCLCPP_INFO(LOGGER, "Gripper current effort feedback: %f", feedback->effort);
    };

    // Send the goal
    gripper_action_client_ptr_->async_send_goal(shared_gripper_data_.gripper_goal, goal_options);

    // Wait for the result
    {
        std::unique_lock<std::mutex> lock(shared_gripper_data_.m);
        shared_gripper_data_.cv.wait(lock, [this]() {
            return shared_gripper_data_.gripper_result != nullptr;
        });
    }

    // Log the final position
    RCLCPP_INFO(LOGGER, "End position: %f", shared_gripper_data_.end_position);

	/* MAIN RETURN PART */
	
	// Determine the success of the gripper operation
    if ((do_grasp && shared_gripper_data_.end_position > 0.16) || 
        (!do_grasp && shared_gripper_data_.end_position < 0.2)) {
        RCLCPP_INFO(LOGGER, "Gripper goal probably reached");
        return moveit::core::MoveItErrorCode::SUCCESS;
    } else {
        RCLCPP_ERROR(LOGGER, "Gripper goal probably not reached!");
        return moveit::core::MoveItErrorCode::FAILURE;
    }

	/*
	//TODO:

	// if(gripper_result->reached_goal) {
	// 	RCLCPP_INFO(LOGGER, "Gripper goal reached");
	// 	return moveit::core::MoveItErrorCode::SUCCESS;
	// } else {
	// 	RCLCPP_ERROR(LOGGER, "Gripper goal not reached!");
	// 	return moveit::core::MoveItErrorCode::FAILURE;
	// }

	*/
	
	// if(gripper_result!=NULL){
	// RCLCPP_INFO_STREAM(LOGGER, gripper_result->reached_goal);
		
	// 	return gripper_result->reached_goal;
	// }
	// else
	// {
		// return false;
	// }
}

bool ManipulatorInterface::move_manipulator(geometry_msgs::msg::PoseStamped &pose_stamped)
{
	std::vector<geometry_msgs::msg::Pose> pick_poses;

	pick_poses = create_pick_moves(pose_stamped.pose);

	success = cartesian_goal(pick_poses[1]);

	return success;
}

bool ManipulatorInterface::move_manipulator(geometry_msgs::msg::Pose &pose)
{
	// std::vector<geometry_msgs::msg::Pose> pick_poses;

	success = cartesian_goal(pose);

	return success;
}

bool ManipulatorInterface::move_manipulator(std::string pose_name)
{
	success = predefined_pose(pose_name);

	return success;
}

// ACTIONS

// geometry_msgs::msg::Pose ManipulatorInterface::find_in(edi_robot_msgs::msg::Embedding &embedding)
// {
// 	geometry_msgs::msg::Pose centre_pose, transformed_pose;

// 	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("semantic_server_client");
// 	rclcpp::Client<edi_robot_msgs::srv::SemanticFindObject>::SharedPtr client =
//     	node_->create_client<edi_robot_msgs::srv::SemanticFindObject>("semantic_object_centroid");

// 	auto request = std::make_shared<edi_robot_msgs::srv::SemanticFindObject::Request>();
// 	// request->embedding = action_desc.object_desc.embedding;
// 	request->embedding = embedding;

// 	while (!client->wait_for_service(1s)) {
//     	if (!rclcpp::ok()) {
//       		RCLCPP_ERROR(LOGGER, "[find_in] Interrupted while waiting for the service. Exiting.");
//       		exit(0);
//     	}
//     	RCLCPP_INFO(LOGGER, "[find_in] service not available, waiting again...");
//   	}

// 	auto result = client->async_send_request(request);
//   	RCLCPP_INFO(LOGGER, "[find_in] JUST SENT A SERVICE REQUEST");

// 	while(rclcpp::spin_until_future_complete(node, result, std::chrono::milliseconds(500)) != rclcpp::FutureReturnCode::SUCCESS)
// 	{
// 		RCLCPP_INFO(LOGGER, "[find_in] Waiting on future for 5s...");
// 	}

// 	RCLCPP_INFO(LOGGER, "[find_in] Service returned a value");

// 	centre_pose = result.get()->centre_pose;

// 	if (centre_pose.position.x == 304.0)
// 	{
// 		RCLCPP_ERROR(LOGGER, "Received centre_pose x value: %f == None type from server", centre_pose.position.x);
// 		transformed_pose = centre_pose;
// 	} else {
// 		RCLCPP_INFO(LOGGER, "[find_in] Returned value is valid, transforming!");
// 		transformed_pose = transform_pose("world", "base_link", centre_pose, "Centre_pose");
// 	}

// 	return transformed_pose;
// }

bool ManipulatorInterface::add_collision_object(geometry_msgs::msg::Pose pose, std::string frame_id, moveit_msgs::msg::CollisionObject &coll_obj){

	bool success;
	// moveit_msgs::msg::CollisionObject collision_object;
	// collision_object.header.frame_id = frame_id;
	coll_obj.header.frame_id = frame_id;
	// collision_object.id = "object";
	coll_obj.id = "object";
	shape_msgs::msg::SolidPrimitive primitive;

	// Define the size of the box in meters
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[primitive.BOX_X] = 0.05;
	primitive.dimensions[primitive.BOX_Y] = 0.05;

	double table_h = 0.92;

	// RCLCPP_INFO(LOGGER, "[add_collision_object] pose.position.z: %f", pose.position.z);
	
	if((pose.position.z - table_h) > 0.0){
		if((pose.position.z - table_h) < 0.06){
			// RCLCPP_INFO(LOGGER, "[add_collision_object] pose.position.z - %f < 0.06, z = %f", table_h, pose.position.z - table_h);
			primitive.dimensions[primitive.BOX_Z] = pose.position.z - table_h;
		} else {
			// RCLCPP_INFO(LOGGER, "[add_collision_object] pose.position.z - %f >= 0.06, z = 0.06", table_h);
			primitive.dimensions[primitive.BOX_Z] = 0.06;
		}
	} else {
		RCLCPP_ERROR(LOGGER, "[add_collision_object] Collision object height value is negative");
		success = false;
		return success;
	}
	pose.position.z = pose.position.z - primitive.dimensions[primitive.BOX_Z]/2;

	// collision_object.primitives.push_back(primitive);
	coll_obj.primitives.push_back(primitive);

	// RCLCPP_INFO(LOGGER, "[add_collision_object] Object height value: %f", collision_object.primitives[0].dimensions[2]);
	RCLCPP_INFO(LOGGER, "[add_collision_object] Object height value: %f", coll_obj.primitives[0].dimensions[2]);

	// collision_object.primitive_poses.push_back(pose);
	coll_obj.primitive_poses.push_back(pose);
	// collision_object.operation = collision_object.ADD;
	coll_obj.operation = coll_obj.ADD;


	// planning_scene_interface.applyCollisionObject(collision_object);
	planning_scene_interface.applyCollisionObject(coll_obj);

	success = true;
	// return collision_object;
	return success;

}

void ManipulatorInterface::attach_collision_object(moveit_msgs::msg::CollisionObject &collision_object){

	// remove_collision_object(collision_object);
	// remove_collision_object();

	// collision_object.operation = collision_object.ADD;

	// attached coll. object start
	moveit_msgs::msg::AttachedCollisionObject attached_collision_object;

	attached_collision_object.link_name = "virtual_ee_link";
	attached_collision_object.touch_links.push_back("right_inner_finger_pad");
	attached_collision_object.touch_links.push_back("left_inner_finger_pad");

	attached_collision_object.object = collision_object;
	// attached coll. object end

	planning_scene_interface.applyAttachedCollisionObject(attached_collision_object);
}

void ManipulatorInterface::detach_collision_object(){

	moveit_msgs::msg::CollisionObject collision_object;
	moveit_msgs::msg::AttachedCollisionObject attached_collision_object;
	// collision_object.id = "object";
	collision_object.operation = collision_object.REMOVE;

	attached_collision_object.object = collision_object;
	attached_collision_object.link_name = "virtual_ee_link";
	planning_scene_interface.applyAttachedCollisionObject(attached_collision_object);
	// planning_scene_interface.applyCollisionObject(collision_object);
}

void ManipulatorInterface::remove_collision_object(){

	std::vector<std::string> object_ids;
	object_ids.push_back("object");
	
	planning_scene_interface.removeCollisionObjects(object_ids);
	// planning_scene_interface.applyPlanningScene();
}

double ManipulatorInterface::get_collision_object(std::string object_id){

	// std::vector<std::string> object_ids;
	// object_ids.push_back("object");
	std::vector<std::string> object_ids = {object_id};

	double z_value = 0.0;

	RCLCPP_INFO(LOGGER, "[get_collision_object] .getObjects(object_ids)");
	// std::map<std::string, moveit_msgs::msg::CollisionObject> object = planning_scene_interface.getObjects();//object_ids);

	std::map<std::string, moveit_msgs::msg::AttachedCollisionObject> attached_objects = planning_scene_interface.getAttachedObjects(object_ids);

	auto it = attached_objects.find(object_id);

    if (it != attached_objects.end()) {
        // Access the AttachedCollisionObject
        const moveit_msgs::msg::AttachedCollisionObject& attached_obj = it->second;

        // Access the CollisionObject within the AttachedCollisionObject
        const moveit_msgs::msg::CollisionObject& obj = attached_obj.object;

        // Check if there are any primitives in the object
        if (!obj.primitives.empty()) {
            // Access the first SolidPrimitive
            const shape_msgs::msg::SolidPrimitive& primitive = obj.primitives[0];

            // Check if the primitive has the dimensions array populated
            if (primitive.dimensions.size() > 2) {
                // Access a specific dimension, e.g., the third dimension (z value)
                z_value = primitive.dimensions[2];
                RCLCPP_WARN(LOGGER, "[get_collision_object] Attached object Z value: %f", z_value);
            } else {
                RCLCPP_ERROR(LOGGER, "[get_collision_object] Primitive does not have enough dimensions.");
            }
        } else {
            RCLCPP_ERROR(LOGGER, "[get_collision_object] No primitives found in the attached object.");
        }
    } else {
        RCLCPP_ERROR(LOGGER, "[get_collision_object] Attached object with ID %s not found.", object_id.c_str());
    }

	return z_value;
}

geometry_msgs::msg::Pose ManipulatorInterface::get_grasp_pose()
{
	geometry_msgs::msg::Pose grasp_pose, transformed_pose;

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("grasp_server_client");
	rclcpp::Client<edi_robot_msgs::srv::GetGraspPose>::SharedPtr client =
    	node_->create_client<edi_robot_msgs::srv::GetGraspPose>("grasp_pose");

	auto request = std::make_shared<edi_robot_msgs::srv::GetGraspPose::Request>();

	while (!client->wait_for_service(1s)) {
    	if (!rclcpp::ok()) {
      		RCLCPP_ERROR(LOGGER, "[get_grasp_pose] Interrupted while waiting for the service. Exiting.");
      		exit(0);
    	}
    	RCLCPP_INFO(LOGGER, "[get_grasp_pose] service not available, waiting again...");
  	}

	auto result = client->async_send_request(request);
  	RCLCPP_INFO(LOGGER, "[get_grasp_pose] JUST SENT A SERVICE REQUEST");

	while(rclcpp::spin_until_future_complete(node, result, std::chrono::milliseconds(500)) != rclcpp::FutureReturnCode::SUCCESS)
	{
		RCLCPP_INFO(LOGGER, "[get_grasp_pose] Waiting on future for 5s...");
	}

	RCLCPP_INFO(LOGGER, "[get_grasp_pose] Service returned a value");

	grasp_pose = result.get()->pose;

	// RCLCPP_INFO(LOGGER, "Received grasp_pose: %f", grasp_pose.position.x);

	if (grasp_pose.position.x == 304.0)
	{
		RCLCPP_ERROR(LOGGER, "Received grasp_pose x value: %f == None type from server", grasp_pose.position.x);
		transformed_pose = grasp_pose;
	} else {
		RCLCPP_INFO(LOGGER, "[get_grasp_pose] Returned value is valid, transforming!");
		transformed_pose = transform_pose("world", "base_link", grasp_pose, "Grasp_pose");
	}

	return transformed_pose;	
}

// DEMO ACTIONS

bool ManipulatorInterface::pick_up(geometry_msgs::msg::Pose &pick_pose)
{
	// bool debug = true;

	world_marker->prompt("press 'Next' to start the pick_up action");
	// RCLCPP_INFO(LOGGER, "Grasping object: %s", action_desc.object_desc.object.data.c_str());

	if(debug){
		world_marker->prompt("press 'Next' to open the gripper");
	}
	
    if(activate_gripper(false) != moveit::core::MoveItErrorCode::SUCCESS){
      RCLCPP_ERROR(LOGGER, "pick_up action failed!");
      return 0;
	} else {
      RCLCPP_INFO(LOGGER, "Gripper opened!");
    }

	if(debug){
		world_marker->prompt("press 'Next' to get object grasp pose");
	}
	geometry_msgs::msg::Pose grasp_pose = get_grasp_pose();
	if (grasp_pose.position.x == 304.0){
		RCLCPP_ERROR(LOGGER, "Pick up action failed!");
		return 0;
	}

	if(debug){
		world_marker->prompt("press 'Next' to add collision object");
	}
	moveit_msgs::msg::CollisionObject coll_obj;
	// success = add_collision_object(action_desc.object_desc.grasp_pose, "world", coll_obj);
	success = add_collision_object(pick_pose, "world", coll_obj);
	if(!success){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug){
		world_marker->prompt("press 'Next' to move above workspace");
	}
	success = predefined_pose("classification");
	if(!success){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}
	
	if(debug){
		world_marker->prompt("press 'Next' to create pick moves for the object");
	}
	// std::vector<geometry_msgs::msg::Pose> pick_poses = create_pick_moves(action_desc.object_desc.grasp_pose);
	std::vector<geometry_msgs::msg::Pose> pick_poses = create_pick_moves(pick_pose);

	check_pose(pick_poses);

	success = cartesian_goal(pick_poses[0]);
	if(!success){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	// RCLCPP_INFO(LOGGER, "pick pose z value before checking %f", pick_poses[1].position.z);

	// check_depth(pick_poses[1]);

	// RCLCPP_INFO(LOGGER, "pick pose z value after checking %f", pick_poses[1].position.z);

	success = cartesian_goal(pick_poses[1]);
	if(!success){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug){
		world_marker->prompt("press 'Next' to attach collision object");
	}
	attach_collision_object(coll_obj);

	if(debug){
		world_marker->prompt("press 'Next' to close the gripper");
	}

	if(activate_gripper(true) != moveit::core::MoveItErrorCode::SUCCESS){
      RCLCPP_ERROR(LOGGER, "Pick up action failed!");
      return 0;
	} else {
      RCLCPP_INFO(LOGGER, "Gripper closed!");
    }

	success = cartesian_goal(pick_poses[0]);
	if(!success){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug){
		world_marker->prompt("press 'Next' to move above workspace");
	}
	success = predefined_pose("classification");
	if(!success){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	success = predefined_pose("wait_outside");
	if(!success){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	return true;
}

bool ManipulatorInterface::put_down(geometry_msgs::msg::Pose &place_pose)
{
	world_marker->prompt("press 'Next' to start the put_down action");

	// if(debug){
	// 	world_marker->prompt("press 'Next' to find objective in scene");
	// }

	// action_desc.object_desc.embedding = embed(action_desc.object_desc.object);

	// if(debug){
	// 	world_marker->prompt("press 'Next' to find objective coordinates");
	// }
	
	// action_desc.object_desc.centre_pose = find_in(action_desc.object_desc.embedding);
	// if (action_desc.object_desc.centre_pose.position.x == 304.0){
	// 	RCLCPP_ERROR(LOGGER, "Put down action failed!");
	// 	return 0;
	// }

	if(debug){
		world_marker->prompt("press 'Next' to get and show object parameters");
	}

	double obj_z = get_collision_object();

	if(debug){
		world_marker->prompt("press 'Next' to move above workspace");
	}
	success = predefined_pose("classification");
	
	if(debug){
		world_marker->prompt("press 'Next' to create pick moves for the object");
	}
	// std::vector<geometry_msgs::msg::Pose> place_poses = create_pick_moves(action_desc.object_desc.centre_pose);
		std::vector<geometry_msgs::msg::Pose> place_poses = create_pick_moves(place_pose);

	check_pose(place_poses, obj_z, true);

	success = cartesian_goal(place_poses[0]);
	if(!success){
		RCLCPP_ERROR(LOGGER, "Put down action failed!");
	}

	success = cartesian_goal(place_poses[1]);
	if(!success){
		RCLCPP_ERROR(LOGGER, "Put down action failed!");
	}

	if(debug){
		world_marker->prompt("press 'Next' to open the gripper");
	}

	if(activate_gripper(false) != moveit::core::MoveItErrorCode::SUCCESS){
      RCLCPP_ERROR(LOGGER, "Place action failed!");
      return 0;
	} else {
      RCLCPP_INFO(LOGGER, "Gripper opened!");
	  detach_collision_object();
    }

	if(debug){
		world_marker->prompt("press 'Next' to remove collision object");
	}
	remove_collision_object();

	success = cartesian_goal(place_poses[0]);
	if(!success){
		RCLCPP_ERROR(LOGGER, "Put down action failed!");
	}

	if(debug){
		world_marker->prompt("press 'Next' to move above workspace");
	}
	success = predefined_pose("classification");
	if(!success){
		RCLCPP_ERROR(LOGGER, "Put down failed!");
		return 0;
	}

	success = predefined_pose("wait_outside");
	if(!success){
		RCLCPP_ERROR(LOGGER, "Put down failed!");
		return 0;
	}
	return true;
}

// ADITTIONAL DEMO FUNCTIONS

void ManipulatorInterface::check_pose(std::vector<geometry_msgs::msg::Pose>& poses, double added_height, bool is_place)
{
	// world_marker->prompt("press 'Next' to delete previous markers");
	RCLCPP_INFO(LOGGER, "[check_pose] checking pose quality");
	world_marker->deleteAllMarkers();
	world_marker->trigger();

	// check depth start
	// double depth_limit = 0.95;
	RCLCPP_INFO(LOGGER, "added_height: %f", added_height);
	double depth_max_pick, depth_max_pre;

	if(is_place){
		depth_max_pick = poses[0].position.z + added_height;
		depth_max_pre = depth_max_pick + 0.04;
		RCLCPP_INFO(LOGGER, "[check_pose] (is_place) Current goal depth limit = %f", depth_max_pick);
		RCLCPP_INFO(LOGGER, "[check_pose] (is_place) Current pre/post depth limit = %f", depth_max_pre);
	} else {
		depth_max_pick = 0.95;// + added_height;
		depth_max_pre = depth_max_pick + 0.04;
		RCLCPP_INFO(LOGGER, "[check_pose] Current goal depth limit = %f", depth_max_pick);
		RCLCPP_INFO(LOGGER, "[check_pose] Current pre/post depth limit = %f", depth_max_pre);
	}

	// double depth_limit = 0.95 + added_height;

	// RCLCPP_INFO(LOGGER, "Current pick depth limit = %f", depth_max_pick);
	// RCLCPP_INFO(LOGGER, "Current pre/post depth limit = %f", depth_max_pre);

	if(debug){
		world_marker->prompt("press 'Next' to visualise original pose");
		RCLCPP_INFO(LOGGER, "Pregoal pose before correcting");
		world_marker->publishAxisLabeled(poses[0], "orig_yaw");
		world_marker->trigger();
	}

	RCLCPP_INFO(LOGGER, "pre/post pose z value before checking: %f", poses[0].position.z);
	if (poses[0].position.z < depth_max_pre) {

		poses[0].position.z = depth_max_pre;
		poses[2].position.z = depth_max_pre;
		RCLCPP_INFO(LOGGER, "pre/post pose z value changed to %f", poses[0].position.z);
	}

	RCLCPP_INFO(LOGGER, "goal pose z value before checking: %f", poses[1].position.z);

	if (poses[1].position.z < depth_max_pick) {

		poses[1].position.z = depth_max_pick;
		RCLCPP_INFO(LOGGER, "goal pose z value changed to %f", poses[1].position.z);
	}

	// check depth end

	// check rotation start

	geometry_msgs::msg::Pose pose = poses[0];

	tf2::Quaternion q(pose.orientation.x, 
					  pose.orientation.y, 
					  pose.orientation.z, 
					  pose.orientation.w);

	tf2::Matrix3x3 m(q);

	double roll, pitch, yaw;
	// double roll_max = 0.2618;
	// double pitch_max = 0.2618;
	double yaw_max = 1.57;

	m.getRPY(roll, pitch, yaw);

	RCLCPP_INFO(LOGGER, "roll: %f", roll);
	RCLCPP_INFO(LOGGER, "pitch: %f", pitch);
	RCLCPP_INFO(LOGGER, "yaw: %f", yaw);

	// if(abs(pitch) > pitch_max){
	// 	if(pitch > 0){
	// 		pitch = pitch_max;
	// 	} else {
	// 		pitch = -pitch_max;
	// 	}
	// }

	// if(abs(roll) > roll_max){
	// 	if(roll > 0){
	// 		roll = roll_max;
	// 	} else {
	// 		roll = -roll_max;
	// 	}
	// }

	if(yaw > yaw_max){
		yaw = yaw - 3.14;
	}

	if(yaw < -yaw_max){
		yaw = yaw + 3.14;
	}

	q.setRPY(roll, pitch, yaw);

	RCLCPP_INFO(LOGGER, "yaw corrected: %f", yaw);

	poses[0].orientation.x = q.getX();
	poses[0].orientation.y = q.getY();
	poses[0].orientation.z = q.getZ();
	poses[0].orientation.w = q.getW();

	poses[1].orientation.x = q.getX();
	poses[1].orientation.y = q.getY();
	poses[1].orientation.z = q.getZ();
	poses[1].orientation.w = q.getW();

	if(debug){
		world_marker->prompt("press 'Next' to visualise corrected yaw");
	}
	RCLCPP_INFO(LOGGER, "[check_pose] Pregoal pose after correcting.");// z value %f", pick_poses[0].position.z);
    world_marker->publishAxisLabeled(poses[0], "corr_pose");
	world_marker->trigger();

	// check rotation end
}

// void ManipulatorInterface::move_in_env()
// {
// 	auto syst_dimensions = rcl_interfaces::msg::Parameter();

// 	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("param_info_client");
// 	rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client =
//     	node_->create_client<rcl_interfaces::srv::GetParameters>("parameters_info");

// 	auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

// 	syst_dimensions.name = "system_dimensions";
// 	request->names.push_back(syst_dimensions.name);


// 	while (!client->wait_for_service(1s)) {
//     	if (!rclcpp::ok()) {
//       		RCLCPP_ERROR(LOGGER, "[param_info_client] Interrupted while waiting for the service. Exiting.");
//       		exit(0);
//     	}
//     	RCLCPP_INFO(LOGGER, "[param_info_client] service not available, waiting again...");
//   	}

// 	auto result = client->async_send_request(request);
//   	RCLCPP_INFO(LOGGER, "[param_info_client] JUST SENT A SERVICE REQUEST");

// 	auto status = result.wait_for(30s);
//   	if (status == std::future_status::ready){}

// 	if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
//   	{
// 		auto param_value = result.get()->values;
// 		syst_dimensions.value = param_value[0];
// 		RCLCPP_INFO(LOGGER, "[param_info_client] 3D is set to %s", syst_dimensions.value.bool_value ? "true" : "false");

// 	} else {
//     	RCLCPP_ERROR(LOGGER, "Failed to call service parameters_info");
//   	}

// 	if(syst_dimensions.value.bool_value){
// 		if(debug){
// 			world_marker->prompt("press 'Next' to go to wait_slam position");
// 		}
// 		success = predefined_pose("wait_slam");
// 	}
// 	else if (!syst_dimensions.value.bool_value)	{
// 		if(debug){
// 			world_marker->prompt("press 'Next' to go to wait_outside position");
// 		}
// 		success = predefined_pose("wait_outside");
// 	}

// 	RCLCPP_INFO(LOGGER, "Waiting for 100ms");
// 	rclcpp::sleep_for(std::chrono::milliseconds(100));
// }

void ManipulatorInterface::map_env()
{
	RCLCPP_INFO(LOGGER, "Moving to 'wait' pose for mapping");
	predefined_pose("wait");

	world_marker->prompt("press 'Next' to start env mapping");

	predefined_pose("scan_0");
	rclcpp::sleep_for(std::chrono::seconds(10));
	predefined_pose("scan_1");
	rclcpp::sleep_for(std::chrono::seconds(10));
	predefined_pose("scan_2");
	rclcpp::sleep_for(std::chrono::seconds(10));
	predefined_pose("scan_3");
	rclcpp::sleep_for(std::chrono::seconds(10));

	world_marker->prompt("press 'Next' to finish env mapping");

	predefined_pose("wait_prompt");

	RCLCPP_INFO(LOGGER, "Mapping complete");
}

} // namespace manipulator_interface