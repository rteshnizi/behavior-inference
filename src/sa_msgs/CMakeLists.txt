cmake_minimum_required(VERSION 3.8)
project(sa_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sa_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose.msg"
  "msg/Velocity.msg"
  "msg/Fov.msg"
  "msg/RobotState.msg"
  "msg/Detection.msg"
  "msg/PoseEstimation.msg"
  "msg/DetectionArray.msg"
  "msg/EstimationMsg.msg"
  "msg/Plan.msg"
  "msg/PoseArray.msg"
  "msg/VisibilityArray.msg"
  "msg/FeatureInfo.msg"
  "srv/QueryVisibility.srv"
 )

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



# add_executable(trajectory_sender src/trajectory_sender.cpp)
# ament_target_dependencies(trajectory_sender rclcpp sa_msgs)         # CHANGE

# add_executable(trajectory_listener src/trajectory_listener.cpp)
# ament_target_dependencies(trajectory_listener rclcpp sa_msgs)         # CHANGE

# install(TARGETS
#   trajectory_sender
#   trajectory_listener
#   DESTINATION lib/${PROJECT_NAME})

ament_package()
