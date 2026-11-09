# Geometry of a rendered or simulated sonar, the sidecar of an image topic (as camera_info is of a camera image).
# Frame: header.frame_id is the sensor frame (x forward, y left, z up).
std_msgs/Header header

uint8 KIND_SIDESCAN=0
uint8 KIND_MULTIBEAM=1
uint8 KIND_IMAGING=2

uint8 kind

# Range limits and resolution along the ray, m.
float64 min_range
float64 max_range
float64 range_resolution

# Beam widths, rad: across and along the sensor's x axis.
float64 horizontal_beamwidth
float64 vertical_beamwidth

# Pointing of the beam centre below the horizontal plane of the sensor frame, rad, positive down; for a pair of
# side-scan transducers the tilt of each.
float64 tilt

# Beams per ping and samples per beam of the image.
uint32 beam_count
uint32 samples_per_beam

# Speed of sound used to turn time into range, m/s.
float64 sound_speed
