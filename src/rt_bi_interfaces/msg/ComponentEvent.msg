string type       # A, D, M, or S for appear, disappear, merge, or split, respectively.
string[] before # The origin of the change, must be set for D, M, and S
string[] after  # The destination of the change, must be set for A, M, and S
int64 ros_time  # The timestamp of this event in ROS clock.
