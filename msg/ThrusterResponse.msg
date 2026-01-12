# ===== existing fields =====
std_msgs/Header header
string frame_id

float32 d
float32 rho
float32 n_tau
float32 n_rate_limit
float32 n_max
string inflow_model
float32[] kt_poly
float32[] kq_poly
float32[2] j_clip

# optional motor
string motor_mode
float32 motor_v
float32 motor_r
float32 motor_ke
float32 motor_i_max

# ===== live command & control =====
float32 cmd_rpm        # rpm actually applied
float32 u_des          # desired surge speed [m/s]
float32 u_error        # PI: speed error [m/s]
float32 n_raw          # PI: raw rpm (pre-cap)
float32 n_sat          # PI: saturated rpm (post-cap)

# ===== thruster telemetry =====
float32 n_rpm          # measured/estimated shaft rpm
float32 j              # advance ratio
float32 kt             # thrust coefficient
float32 kq             # torque coefficient
float32 t              # thrust [N]
float32 q              # torque [N·m]
float32 p_shaft        # shaft power [W]
float32 p_elec         # electrical power [W]
float32 eta_prop       # prop efficiency
float32 eta_motor      # motor efficiency
float32 eta_overall    # overall efficiency
float32 i              # motor current [A]
float32 v              # motor effective voltage [V]

