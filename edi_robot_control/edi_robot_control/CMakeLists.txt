cmake_minimum_required(VERSION 3.8)
project(edi_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rosidl_runtime_cpp REQUIRED)
find_package(edi_robot_msgs REQUIRED)
find_package(pick_ik REQUIRED)
find_package(control_msgs REQUIRED)
find_package(manipulator_interface REQUIRED)
#find_package(primitives_interface REQUIRED)


add_executable(edi_pick_and_place src/edi_pick_and_place.cpp 
				  # src/trajectory_generation.cpp 
				  # src/detect_object.cpp 
				  # src/cartesian_goal.cpp 
				  # src/predefined_pose.cpp
				  # src/get_named_object_pose.cpp
				  # src/create_manipulation_poses.cpp
				  # src/create_pick_moves.cpp
				  # src/set_gripper.cpp
				  # src/utilities/manipulator_interface.cpp
		)
		
add_executable(object_server src/object_server.cpp)
ament_target_dependencies(object_server rclcpp edi_robot_msgs)

add_executable(object_server_client src/object_server_client.cpp)
ament_target_dependencies(object_server_client rclcpp edi_robot_msgs)
		
target_include_directories(edi_pick_and_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(edi_pick_and_place PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  edi_pick_and_place
  #rosidl_runtime_cpp
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "edi_robot_msgs"
  "control_msgs"
  "manipulator_interface"
  #primitives_interface
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS 
    edi_pick_and_place
    object_server
    object_server_client
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY 
    launch 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
