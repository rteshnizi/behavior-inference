# Unique string id
string id

# The timestamp associated with the update.
# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs') # CSpell: ignore nsec, nsecs
# time-handling sugar is provided by the client library
builtin_interfaces/Time stamp

string STATIC = "static"
string DYNAMIC = "dynamic"
string AFFINE = "affine"
string space_type # One of the above constants

# Symbolized region predicates
rt_bi_interfaces/Predicate[] predicates

# A list of polygons.
rt_bi_interfaces/Polygon[] polygons

######################### Properties #########################
# Name of the object. It is not necessarily unique.
string name
string color
# Traversability(legs: bool, wheels: bool, swim: bool)
rt_bi_interfaces/Traversability traversability
# Dynamic spaces only: times during which this set is accessible
# TimeInterval(start: builtin/Time, end: builtin/Time, include_left: bool, include_right: bool)
rt_bi_interfaces/TimeInterval[] accessible_times
# Affine spaces only: the fully qualified name of the associated ROS node.
string ros_node
