# One system a fleet bridge has heard on the vehicle network.
uint16 source             # the system's address on its own network
string sys_name           # its announced name
string topic_namespace    # the ROS namespace of its topics
bool announced            # it has announced itself
bool has_udp_endpoint     # its announcement gave an address the bridge can send to
string services           # the services it announced
string udp_host
uint16 udp_port
float32 last_seen_age_sec # seconds since it was last heard
