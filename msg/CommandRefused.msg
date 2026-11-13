# A command the bridge did not send to a vehicle, and why. Published by the bridge between the ROS graph and a vehicle's own
# message protocol, one per refused message.

string CHECK_NEVER_SEND="never_send"        # on the never-send list and not unlocked
string CHECK_CONTROL_KEY="control_key"      # a reference-following command that does not name this bridge as controller
string CHECK_ALLOW_LIST="allow_list"        # no command topic of this bridge sends it to this vehicle
string CHECK_VERSION="version"              # the vehicle's message definition is unknown, lacks the message or differs
string CHECK_CONVERSION="conversion"        # the ROS message could not be converted
string CHECK_SIZE="size"                    # the packet is over a size limit
string CHECK_TRANSPORT="transport"          # the link refused it

std_msgs/Header header
string message_name      # the refused message's name in the vehicle protocol, e.g. PlanControl
uint16 message_id        # its number in that protocol; 0 when it was refused before one was known
string target_vehicle    # the bridge's target vehicle (its announced name); empty when the bridge names none
uint16 target_address    # the vehicle address the message was for; 0 when not resolved yet
string check             # one of the CHECK_ values
string never_send_entry  # the never-send entry that refused it, when check is never_send; else empty
string reason            # the full reason, with the place inside a carried message
