# message for contact frames and wrenches that we measure from the simulation
# all data in world frame by default!

# contact frame x axis points along contact normal and (y,z) form a right-handed coosy 
geometry_msgs/Transform contact_frame

# this is already included in contact_frame, but still publishing for completeness
geometry_msgs/Vector3 contact_position 
geometry_msgs/Vector3 contact_normal

# on which Reflex finger is the contact (ranges from 0 to 3, 0 means palm contact) 
int32 finger_id

# to which Reflex sensor would this contact belong (ranges from 0 to 9, 0 means palm contact) 
int32 sensor_id

# whether the contact is on the palm (contact is either on finger or palm)
bool palm_contact

# measured contact wrench in world frame
geometry_msgs/Wrench contact_wrench

# magnitudes of contact wrench
float64 contact_force_magnitude
float64 contact_torque_magnitude