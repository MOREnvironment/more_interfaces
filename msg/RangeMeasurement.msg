# One range measurement from a sensor mounted on a vehicle (echo sounder, altimeter, acoustic range to a beacon).
# Units: metres for lengths, radians for angles.
# Frames: header.frame_id names the frame that pose is expressed in; for a sensor on a vehicle it is the vehicle body frame
# (ROS body axes: x forward, y left, z up).

std_msgs/Header header

# Source frame (duplicated from header.frame_id for UI convenience).
string frame_id

# Validity copied from the upstream sensor or protocol: 0 = invalid, 1 = valid.
uint8 validity

# Sensor pose in header.frame_id: position in m, orientation as a unit quaternion.
geometry_msgs/Pose pose

# Beam footprint of the sensor in rad. Not every source reports it; the publisher's documentation says what it sends then.
float32 beam_width
float32 beam_height

# Measured range in m.
float32 range
