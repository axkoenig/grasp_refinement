# message for poses of all 27 tactile sensors on Reflex hand
std_msgs/Header header
string child_frame_id

# order is finger_1, finger_2, finger_3
geometry_msgs/Point[27] sensor_position
geometry_msgs/Vector3[27] sensor_normal
