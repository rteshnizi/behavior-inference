# A unique id associated with the detected moving body.
string id
# Information about dynamic's of a moving body
geometry_msgs/Pose pose

# The props below are commented since this is not used here. But they are part of the design
# geometry_msgs/Pose velocity
# Confidence of detection, a 4x4 matrix of covariance of [x, y, vx, vy]
# use np.reshape((4,4)) to reconstruct it
# float32[] covariance

bool entered
bool exited
