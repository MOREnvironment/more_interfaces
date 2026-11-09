# The plans stored on the vehicle. Published latched, and again when the store changes.
std_msgs/Header header
more_interfaces/PlanSummary[] plans

# The plan that is running. Empty: none.
string running_plan_id
