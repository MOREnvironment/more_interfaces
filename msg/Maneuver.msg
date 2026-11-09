# One manoeuvre of a plan. Fields that a kind does not use are ignored and set to NaN (or zero for flags).
# Frames and units: positions WGS-84 (latitude, longitude in degrees; the altitude field of `position` is unused and is
# NaN); vertical references in m, interpreted by z_mode; speeds m/s; radii m; heading and course in rad from true north,
# clockwise; durations and timeouts in s on the host's clock.

uint8 KIND_GO_TO_POINT=0          # position, z, z_mode, speed
uint8 KIND_LOITER=1               # position, z, z_mode, radius, direction, speed, duration
uint8 KIND_STATION_KEEPING=2      # position, z, z_mode, radius, speed, duration
uint8 KIND_SPIRAL_PROFILE=3       # position, start_z -> z, z_mode, radius, speed, use_current_position
uint8 KIND_IDLE=4                 # duration
uint8 KIND_SURFACE_AND_HOLD=5     # position, z, z_mode, radius, speed, duration, use_current_position, hold, end_at_surface
uint8 KIND_HOLD_SETPOINT=6        # heading, z, z_mode, speed, duration
uint8 KIND_BRAKE=7                # duration

uint8 Z_MODE_NONE=0               # no vertical reference
uint8 Z_MODE_DEPTH=1              # positive down from the surface
uint8 Z_MODE_ALTITUDE=2           # positive up from the seabed
uint8 Z_MODE_HEIGHT_ABOVE_ELLIPSOID=3   # positive up from the ellipsoid

uint8 DIRECTION_CLOCKWISE=0
uint8 DIRECTION_COUNTER_CLOCKWISE=1

# Label that the plan's transitions name. Empty: the manoeuvre has no label (it runs in order).
string maneuver_id

uint8 kind

# Centre or target point (kinds that use one).
geographic_msgs/GeoPoint position

# Vertical reference, in the unit z_mode names. For KIND_SPIRAL_PROFILE this is the end of the helix and start_z its
# start (NaN: the depth the vehicle has).
float64 z
float64 start_z
uint8 z_mode

float64 speed
float64 radius
uint8 direction
float64 heading

# The manoeuvre ends this many seconds after it begins (Inf: never).
float64 duration

# The plan fails when the manoeuvre has run this long (Inf: none).
float64 timeout

# Flags of KIND_SPIRAL_PROFILE and KIND_SURFACE_AND_HOLD: take the vehicle's own position as the centre (position then
# ignored); hold the point (not: wait for a good fix); end at the surface (not: return to z).
bool use_current_position
bool hold
bool end_at_surface

more_interfaces/PartAction[] start_actions
more_interfaces/PartAction[] end_actions
