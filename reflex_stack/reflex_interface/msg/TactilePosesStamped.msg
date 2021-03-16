# message for all "tactile poses" that correspont to the 27 sensors on Reflex hand
# the "tactile poses" lie on the surface of each finger above the corresponding sensor 
# (i.e. approximately at the contact location)
# this message is published by reflex interface in the "shell" frame

std_msgs/Header header

# order is [finger_1, finger_2, finger_3]
geometry_msgs/Vector3[3] prox_finger_normal
geometry_msgs/Vector3[3] dist_finger_normal

# order is [9*finger_1, 9*finger_2, 9*finger_3]
geometry_msgs/Point[27] tactile_position