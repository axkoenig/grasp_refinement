# message for all "tactile poses" that correspont to the 27 sensors on Reflex hand
# the "tactile poses" lie on the surface of each finger above the corresponding sensor 
# (i.e. approximately at the contact location)
# this message is published by reflex interface in the "shell" frame

std_msgs/Header header

# order is [finger_1, finger_2, finger_3]
geometry_msgs/Vector3[3] prox_normal
geometry_msgs/Vector3[3] dist_normal

# boolean flags if link is in contact
#order is [finger_1, finger_2, finger_3]
bool[3] prox_in_contact
bool[3] dist_in_contact
bool[3] finger_in_contact

# order is [9*finger_1, 9*finger_2, 9*finger_3]
geometry_msgs/Point[27] tactile_position

# for sake of completeness we also re-publish the sensor readings from reflex driver
bool[27] sensor_contact
float[27] sensor_pressure