# A position of a target located by a USBL head, as the observer reports it.
# Frame: WGS-84. header.stamp = the time the message was produced, on the host's clock; the time of the measurement
# itself (the modem stamp corrected to the instant of emission) is `measurement_time`.

std_msgs/Header header

uint8 Z_NONE=0
uint8 Z_DEPTH=1                # z is a depth, m, positive down from the surface
uint8 Z_ALTITUDE=2             # z is an altitude, m, positive up from the seabed
uint8 Z_HEIGHT=3               # z is a height, m, positive up from a datum the reporting unit defines

uint8 ACCURACY_UNKNOWN=0
uint8 ACCURACY_SUBARRAY_SCATTER_STD=1   # standard deviation of the sub-array position estimates about their combined
                                        # solution over recent fixes: random scatter only, never systematic error
uint8 ACCURACY_SIGMA_1=2
uint8 ACCURACY_RADIUS_95=3
uint8 ACCURACY_TIER=4

# Who measured (the surface vehicle) and who is located.
string observer
string target

# Latitude and longitude, degrees. `position.altitude` is always NaN (the vertical coordinate is z and z_reference).
geographic_msgs/GeoPoint position

uint8 z_reference
float64 z

# Accuracy, m, unconverted, with its meaning in accuracy_kind. Never to be used as the whole fix error or put into a
# covariance by a consumer that does not know what it means. NaN: not available.
float64 accuracy
uint8 accuracy_kind

builtin_interfaces/Time measurement_time
bool has_measurement_time
