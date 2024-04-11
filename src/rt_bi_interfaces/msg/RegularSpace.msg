# Unique string id
string id

string STATIC = "static"
string DYNAMIC = "dynamic"
string AFFINE = "affine"
string space_type # One of the above constants

# Symbolized region predicates
rt_bi_interfaces/Predicate[] predicates

# A list of polygons.
rt_bi_interfaces/Polygon[] polygons

# Spatial specs.
rt_bi_interfaces/SpaceSpec spec

# The timestamp associated with the update.
# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs') # CSpell: ignore nsec, nsecs
# time-handling sugar is provided by the client library
builtin_interfaces/Time stamp

# If this is an affine region,
# this would contain the full-name of the associated ROS node.
string ros_node
