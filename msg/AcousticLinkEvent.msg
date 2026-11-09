# One event of an acoustic link: a frame delivered, a transmission reported, a range, a status. Link health is not a
# sensor reading, so it has its own type. header.stamp = the time of the event on the host's clock.
std_msgs/Header header

uint8 KIND_TX_STATUS=0
uint8 KIND_RX_FRAME=1
uint8 KIND_RX_RANGE=2
uint8 KIND_STATUS=3

# Label of the link (modem).
string link

uint8 kind

# Sequence number of the frame, as the link reports it.
uint16 seq

# Status of the event as the link reports it: the code and its name from that link's own definition.
int32 status_code
string status_name

# Peer the event concerns.
string peer

# Range to the peer, m, for KIND_RX_RANGE. NaN: not available.
float64 range

# Size of the frame, bytes.
uint32 bytes

# Level of the received signal, dB, and the link's integrity figure for the frame (as the link reports it, unconverted).
# NaN: not available.
float64 signal_level
float64 integrity
