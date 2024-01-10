# Dynamic region's unique id.
uint32 id
# time associated with the update.
# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs') # CSpell: ignore nsec, nsecs
# time-handling sugar is provided by the client library
builtin_interfaces/Time stamp

geometry_msgs/Pose pose
geometry_msgs/Point center_of_rotation
geometry_msgs/Polygon region
