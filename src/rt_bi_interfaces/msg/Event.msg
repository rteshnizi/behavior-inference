string type                  # A, D, M, or S for appear, disappear, merge, or split, respectively.
rt_bi_interfaces/Id[] before # The src node of the change, expected when type is D, M, or S
rt_bi_interfaces/Id[] after  # The dst node of the change, expected when type is A, M, or S

# New spatial edges. This information is relevant only when a node appears (A)
rt_bi_interfaces/Id[] spatial_edge_from
rt_bi_interfaces/Id[] spatial_edge_to

int64 time_nano_secs # The timestamp of this event in ROS clock.
