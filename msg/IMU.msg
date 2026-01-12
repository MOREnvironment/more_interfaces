# auv_user_interfaces/IMU.msg
# Aggregated IMU report for UI/logging convenience.

std_msgs/Header header

# Source frame (duplicated from header.frame_id for UI convenience)
string frame_id

# Orientation (as quaternion) and Euler (radians)
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 rpy   # roll, pitch, yaw [rad]

# Body angular velocity and linear acceleration
geometry_msgs/Vector3 gyro   # [rad/s]
geometry_msgs/Vector3 accel  # [m/s^2], either specific force or linear acc depending on sim setting

# Norms (magnitude) for quick monitoring
float32 gyro_norm
float32 accel_norm

# Saturation flag (either gyro or accel saturated)
bool saturation

# Stats: [clips_gyro, clips_acc, dropouts]
uint32[3] stats
