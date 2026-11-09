# Mode of the vehicle and why it is in error, if it is.

std_msgs/Header header

# Mode word of the vehicle's interface (for example service, maneuver, external, error).
string mode

# The monitors or parts that judged a breach in the last period (their labels). Empty: none.
string[] parts_in_error

# Reason of the latest breach. Empty: none yet.
string last_error

# Time of the latest breach, seconds on the clock of header.stamp. NaN: none yet.
float64 last_error_time

# Kind of the manoeuvre in progress; one of the Maneuver KIND_* constants, or 255 when none.
uint8 maneuver_kind
