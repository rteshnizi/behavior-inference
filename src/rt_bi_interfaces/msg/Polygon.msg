# Unique string id
string id

# Polygon = { geometry_msgs/Point32[] points }
geometry_msgs/Polygon region


string STATIC = "static"
string DYNAMIC = "dynamic"
string AFFINE = "affine"
string type # One of the above constants

# Only valid if affine is True
geometry_msgs/Point32 center_of_rotation
