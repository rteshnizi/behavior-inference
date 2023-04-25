float32 detection_time

# robot states: position and FoV
sa_msgs/RobotState robot_state

# the estimation information of all trajs
sa_msgs/PoseEstimation[] pose_estimations

# number of objects detected by beam sensor only
int8 discrete_detections