std_msgs/Header header

# target_uav=0 means broadcast.
uint8 target_uav
uint32 mission_id
uint32 command_id
builtin_interfaces/Time valid_until

# "goto", "hold" or "land". Target is in the common swarm NED frame.
string command
geometry_msgs/Point target
