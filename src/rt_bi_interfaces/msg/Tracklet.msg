### Amended message format derived from SaMsg

# A unique id associated with the detected moving body.
uint32 id
# A label to identify the type of the moving object
string category_label
# Information about dynamic's of a moving body
geometry_msgs/Pose pose
geometry_msgs/Pose velocity

# Confidence of detection, a 4x4 matrix of covariance of [x, y, vx, vy]
# use np.reshape((4,4)) to reconstruct it
float32[] covariance

bool spawned
bool vanished
