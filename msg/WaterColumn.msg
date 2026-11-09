# Depth and altitude of an underwater vehicle.
# Frame: header.frame_id is the vehicle's body frame; both values are scalars along the local vertical.

std_msgs/Header header

# Depth below the surface, m, positive down (the navigation estimate).
float64 depth

# Vertical distance to the seabed that the vehicle's own sensor measured, m, positive up from the seabed.
# NaN when no valid measurement is available. Never the true value.
float64 altitude
