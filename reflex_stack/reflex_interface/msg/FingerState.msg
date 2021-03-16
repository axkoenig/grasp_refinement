# message contains "tactile poses" that correspont to the sensors on Reflex hand
# the "tactile poses" lie on the surface of each finger above the corresponding sensor 
# (i.e. approximately at the contact location)
# this message is published by reflex interface in the "shell" frame

geometry_msgs/Vector3 prox_normal
geometry_msgs/Vector3 dist_normal
geometry_msgs/Point[9] tactile_position

# boolean flags if link is in contact
bool prox_in_contact
bool dist_in_contact
bool finger_in_contact

# this data is also available from reflex driver, but we republish it for completeness
bool[9] sensor_contact
float64[9] sensor_pressure
float64 proximal_angle
float64 distal_angle