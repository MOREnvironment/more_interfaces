# State of one part of the vehicle (a monitor, a sensor, an actuator, a payload).

uint8 HEALTH_OK=0
uint8 HEALTH_DEGRADED=1
uint8 HEALTH_FAULT=2
uint8 HEALTH_UNKNOWN=3

# Label of the part.
string part

# Kind of the part (for example sensor, actuator, monitor, payload).
string kind

uint8 health

# For a monitor that judged a breach: the kind of breach, one of min_altitude, max_depth, region, timeout, or another
# word the vehicle's family documents. Empty otherwise.
string text_kind

# The number the breach was judged on (m for min_altitude and max_depth, s for timeout). NaN: none.
float64 value
