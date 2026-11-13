# The state of a bridge's link to one vehicle, published periodically by the bridge between the ROS graph and a vehicle's
# own message protocol.
std_msgs/Header header
string mode                  # how the bridge finds its vehicle, e.g. fixed, dynamic or fleet
bool announced               # the vehicle has announced itself
bool dynamic_target          # the target address was learnt from the vehicle's announcement
bool entity_list_received    # the vehicle answered the bridge's entity list query
uint16 source                # the vehicle's address on its own network
string sys_name              # the vehicle's announced name
string topic_namespace       # the ROS namespace of the vehicle's topics
string target_host           # where the bridge sends
uint16 target_port
uint32 candidate_count       # vehicles announcing the target name
float32 last_seen_age_sec    # seconds since the vehicle was last heard
float32 last_switch_age_sec  # seconds since the target last changed

# Counters and offsets the bridge does not measure yet are published as NOT MEASURED, never as zero (a zero would read as
# "no failures" or "clocks agree").

# Number of received packets that failed to decode since the bridge started. -1: not measured.
int64 decode_failure_count -1

# Observed offset of the vehicle's clock from the bridge host's clock, seconds (bridge receive time minus the packet's own
# time). NaN: not measured.
float64 clock_offset_sec
