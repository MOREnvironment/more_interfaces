# Runtime monitor of a simulated host, published about once a second of wall time (best effort).
# The wall time here is a measurement for this monitor only; no logic of a host or a consumer reads it.

uint8 MODE_REALTIME=0
uint8 MODE_MAX=1
uint8 MODE_STEPPED=2

uint8 STATUS_OK=0
uint8 STATUS_DEGRADED=1           # the host cannot keep the declared factor and has decimated or fallen behind

# Simulation time of the sample, and the wall-clock time it was taken.
builtin_interfaces/Time sim_time
builtin_interfaces/Time wall_time

uint8 mode
uint8 status

# Real-time factor the host was asked for (NaN for max and stepped) and the one achieved over the window.
float64 declared_factor
float64 achieved_factor

# Number of steps the percentiles below cover.
uint32 window_steps

# Step time in seconds of wall time, median and 99th percentile, split by where it went.
float64 step_p50
float64 step_p99
float64 model_step_p50
float64 model_step_p99
float64 publication_p50
float64 publication_p99
float64 ack_wait_p50
float64 ack_wait_p99

# Steps that overran their wall-time slot since the start, and the current lag behind the schedule, seconds of wall
# time (0 when none).
uint64 overrun_count
float64 lag

more_interfaces/ConsumerLatency[] consumers
