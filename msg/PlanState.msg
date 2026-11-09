# State of the plan. Every transition is published (reliable, depth 50), none is dropped.
std_msgs/Header header

# State words of the plan; the first eight are the states of a running plan, STARTING is published once when a plan
# is started.
uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_PAUSED=2
uint8 STATE_RECOVERING=3
uint8 STATE_SUCCEEDED=4
uint8 STATE_FAILED=5
uint8 STATE_ABORTED=6
uint8 STATE_CANCELLED=7
uint8 STATE_STARTING=8

# `outcome` value while the plan has not ended.
uint8 OUTCOME_NONE=255

uint8 state

# The plan. Empty: started without an id.
string plan_id

# The manoeuvre at `index`. Empty: it has none.
string maneuver_id

# Position of the manoeuvre in the plan, and the number of manoeuvres.
uint32 index
uint32 count

# Fraction of the manoeuvres done, 0..1 (1 when the plan has succeeded).
float64 progress

# Final state once the plan has ended (a STATE_ constant), else OUTCOME_NONE.
uint8 outcome

# The plan this one replaced (when state is STATE_STARTING). Empty: none was running.
string previous_plan_id

# Why the plan paused, failed or aborted.
string reason
