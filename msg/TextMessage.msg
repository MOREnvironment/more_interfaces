# A text message as it arrives at (or leaves) the vehicle: an operator command, a status.
std_msgs/Header header         # stamp = time of arrival, on the host's clock

uint8 MEDIUM_ACOUSTIC=0
uint8 MEDIUM_RADIO=1
uint8 MEDIUM_OTHER=2

string text
string sender
uint8 medium
