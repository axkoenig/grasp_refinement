# message for all "tactile poses" that correspont to the 27 sensors on Reflex hand
# the "tactile poses" lie on the surface of each finger above the corresponding sensor 
# (i.e. approximately at the contact location)

std_msgs/Header header
string child_frame_id       # by default in "shell" frame

# order is finger_1, finger_2, finger_3
geometry_msgs/Point[27] sensor_position
geometry_msgs/Vector3[27] sensor_normal
