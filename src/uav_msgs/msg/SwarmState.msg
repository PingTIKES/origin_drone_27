std_msgs/Header header

uint8 uav_id
uint32 seq

# Common swarm NED frame. All vehicles must use the same measured transform.
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 position_variance

bool pose_valid
bool vio_valid
float32 battery_remaining
string mission_state
uint32 frame_epoch
