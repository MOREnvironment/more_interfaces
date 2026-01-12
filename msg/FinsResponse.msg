std_msgs/Header header

# Source frame (duplicated from header.frame_id for UI convenience)
string frame_id

string[] keys
float32[] s
float32[] rho
float32[] a0
float32[] cd0
float32[] k_ind
float32[] alpha_soft_clip
float32[] delta_max
float32[] delta_rate
float32[] tau
float32[] input_delay
float32[] u_deadband
float32[] u_hyst
geometry_msgs/Vector3[] r_b
geometry_msgs/Vector3[] n_lift_b
geometry_msgs/Vector3[] n_flow_b
string[] input_mode