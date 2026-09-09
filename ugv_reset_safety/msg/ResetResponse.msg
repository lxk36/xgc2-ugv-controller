uint8 RUNNING=0
uint8 ARRIVED=1
uint8 REJECTED=2
std_msgs/Header header
uint32 generation
geometry_msgs/Twist command
uint8 status
string reason
