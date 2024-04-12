# Unique string id
string id

# The timestamp associated with the update.
# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs') # CSpell: ignore nsec, nsecs
# time-handling sugar is provided by the client library
builtin_interfaces/Time stamp

# Symbolized region predicates
rt_bi_interfaces/Predicate[] predicates

# The spatial geometry associated with this update
rt_bi_interfaces/Polygon[] polygons
# The temporal ranges associated with this update
rt_bi_interfaces/TimeInterval[] intervals

######################### Properties #########################
# Name of the object. It is not necessarily unique.
string name
string color
# The fully qualified name of the associated ROS node.
string ros_node

string STATIC = "S"
string DYNAMIC = "D"
string AFFINE = "A"
string SENSING = "Z"
string TARGET = "T"
string space_type # One of the above constants
