# The vehicle took an abort. Published once, to every client, whoever sent the order.
std_msgs/Header header

# Label the sender of the order gave (empty: none, or the vehicle itself).
string source

# Why the abort was taken.
string reason
