# State of one payload (a sonar, a camera, a sampler).
std_msgs/Header header

uint8 ACTIVATION_INACTIVE=0
uint8 ACTIVATION_ACTIVATING=1     # switched on, not yet delivering (warm-up delay)
uint8 ACTIVATION_ACTIVE=2
uint8 ACTIVATION_DEACTIVATING=3

uint8 HEALTH_OK=0
uint8 HEALTH_DEGRADED=1
uint8 HEALTH_FAULT=2
uint8 HEALTH_UNKNOWN=3

# Label of the payload (the <name> of the topic).
string name

uint8 activation
uint8 health

# The settings in force.
more_interfaces/Setting[] settings

# The fault text while health is HEALTH_FAULT, else empty.
string fault
