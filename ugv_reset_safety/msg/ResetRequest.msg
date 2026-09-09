# Session generation and exact request timestamp are echoed by the coordinator.
std_msgs/Header header
uint32 generation
time pose_stamp
geometry_msgs/Pose2D pose
geometry_msgs/Pose2D target
# Last command actually emitted by the native owner, not a proposed response.
geometry_msgs/Twist applied_command
time applied_stamp
