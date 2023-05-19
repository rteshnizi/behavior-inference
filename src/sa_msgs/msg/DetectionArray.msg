float32 detection_time

# robot states: position and FoV
sa_msgs/RobotState robot_state

# the observation information of detected_label and position
sa_msgs/Detection[] detections

# sensor type
string sensor_type

# Object detected by beam sensor only
bool beam_detection
