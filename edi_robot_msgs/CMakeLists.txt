cmake_minimum_required(VERSION 3.8)
project(edi_robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionDesc.msg"
  "msg/ObjectDesc.msg"
  "msg/Embedding.msg"
  "srv/GetGraspPose.srv"
  "srv/GetObjectEmbedding.srv"
  "srv/GetActionList.srv"
  "srv/GetCommand.srv"
  "srv/SemanticFindObject.srv"
  "srv/NamedObjectPose.srv"
  "srv/NamedObjectPoseStamped.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
