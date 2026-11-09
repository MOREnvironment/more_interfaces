# Progress of a text message sent with SendText, matched by request_id.
std_msgs/Header header

uint8 STATE_QUEUED=0
uint8 STATE_SENT=1
uint8 STATE_DELIVERED=2           # the peer confirmed reception
uint8 STATE_FAILED=3

uint32 request_id
uint8 state

# Bytes put on the medium.
uint32 bytes

# Why it failed. Empty otherwise.
string reason
