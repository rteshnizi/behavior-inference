cmake_minimum_required(VERSION 3.8)
project(rt_bi_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(sa_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/Adjacency.msg"
  "msg/ColdStart.msg"
  "msg/ComponentEvent.msg"
  "msg/DataRefRequest.msg"
  "msg/DataRefResponse.msg"
  "msg/DynamicRegion.msg"
  "msg/Estimation.msg"
  "msg/EstimationMsg.msg"
  "msg/Events.msg"
  "msg/Graph.msg"
  "msg/MapRegion.msg"
  "msg/Predicate.msg"
  "msg/Polygon.msg"
  "msg/PoseEstimation.msg"
  "msg/RegularSpace.msg"
  "msg/RegularSpaceArray.msg"
  "msg/SpaceSpec.msg"
  "msg/TimeInterval.msg"
  "msg/Tracklet.msg"
  "msg/Traversability.msg"
  # Services
  "srv/DataReference.srv"
  "srv/SpaceTime.srv"
  DEPENDENCIES builtin_interfaces
  DEPENDENCIES geometry_msgs
  DEPENDENCIES sa_msgs
  DEPENDENCIES std_msgs
 )
ament_export_dependencies(rosidl_default_runtime)

ament_package()
