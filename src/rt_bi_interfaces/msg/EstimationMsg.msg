### Amended message format derived from SaMsg

float32 detection_time

# robot states: position and FoV
sa_msgs/RobotState robot_state

# All the detected tracklets
rt_bi_interfaces/PoseEstimation[] pose_estimations

# number of objects detected by beam sensor only
int8 discrete_detections
