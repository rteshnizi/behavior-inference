# class of labeling
string detection_label

# trajectory id
uint16 trajectory_id

# trajectory estimation
sa_msgs/Pose pose
sa_msgs/Velocity velocity

# Confidence of detection, a 4x4 matrix of covariance of [x, y, vx, vy]
# use np.reshape((4,4)) to reconstruct it
float32[] covariance   