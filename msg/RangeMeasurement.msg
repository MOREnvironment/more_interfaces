# One range measurement from a sensor mounted on a vehicle (echo sounder, altimeter, acoustic range to a beacon).
# Units: metres for lengths, radians for angles.
# Frames: header.frame_id names the frame that pose is expressed in; for a sensor on a vehicle it is the vehicle body frame
# (ROS body axes: x forward, y left, z up). It is a frame, never an identifier: what the range was measured to goes in target_id.

std_msgs/Header header

# Identifier of what the range was measured to: for an acoustic range, the beacon. Empty when the sensor ranges to no
# identified target (an echo sounder, an altimeter).
string target_id

# Validity: 0 = invalid, 1 = valid. Copied from the sensor or protocol when it reports validity; a source that reports none
# is documented by the publisher with the rule it applies.
uint8 validity

# Sensor pose in header.frame_id: position in m, orientation as a unit quaternion.
geometry_msgs/Pose pose

# Beam footprint of the sensor in rad. NaN: the source does not report it (zero would be a measurement).
float32 beam_width
float32 beam_height

# Measured range in m.
float32 range
