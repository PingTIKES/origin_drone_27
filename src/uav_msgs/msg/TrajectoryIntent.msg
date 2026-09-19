std_msgs/Header header

uint8 uav_id
uint32 seq
builtin_interfaces/Time valid_until

# Constant-spacing prediction in the common swarm NED frame.
float32 time_step
geometry_msgs/Point[] points
geometry_msgs/Vector3[] velocities
