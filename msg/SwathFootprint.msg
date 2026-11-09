# What a swath sensor (a side-scan or a multibeam) insonified in one ping, on the seabed, as intervals of ground range
# from the nadir. Published per ping, or decimated (see `decimation`).
# Frame: header.frame_id is the sensor frame; ground range is horizontal distance from the nadir point, port and
# starboard as seen looking forward along the sensor's x axis; hit points are expressed in the sensor frame.
# Time: header.stamp is the ping time on the host's clock.

std_msgs/Header header

# Label of the payload or sensor.
string sensor

# Number of pings this message stands for (1: every ping).
uint32 decimation

# Height of the sensor above the seabed under it, m. NaN: not available.
float64 altitude

# Ground ranges insonified by the sensor, per side.
more_interfaces/RangeInterval[] port_coverage
more_interfaces/RangeInterval[] starboard_coverage

# Ground ranges inside the coverage that lie in the shadow of seabed relief or an object.
more_interfaces/RangeInterval[] port_shadow
more_interfaces/RangeInterval[] starboard_shadow

# Ground ranges inside the coverage where the swath saw the surveyed object (a pipe).
more_interfaces/RangeInterval[] port_object
more_interfaces/RangeInterval[] starboard_object

# Points where the rays met the seabed or an object, m, in the sensor frame.
geometry_msgs/Point[] hit_points
