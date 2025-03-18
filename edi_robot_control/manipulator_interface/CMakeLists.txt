cmake_minimum_required(VERSION 3.8)
project(manipulator_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # New, in case of issues, comment out

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(edi_robot_msgs REQUIRED)

add_library(manipulator_interface src/manipulator_interface.cpp)

target_include_directories(manipulator_interface 
    PUBLIC
    #${CMAKE_CURRENT_SOURCE_DIR}/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(manipulator_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#target_compile_definitions(manipulator_interface PRIVATE "MANIPULATOR_INTERFACE_BUILDING_LIBRARY")
target_compile_definitions(manipulator_interface PUBLIC "MANIPULATOR_INTERFACE_BUILDING_LIBRARY")

ament_target_dependencies(
  manipulator_interface PUBLIC
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  edi_robot_msgs
  control_msgs
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS manipulator_interface
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(some_dependency)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(manipulator_interface)

ament_package()
