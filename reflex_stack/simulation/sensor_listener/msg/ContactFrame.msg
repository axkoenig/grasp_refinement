# message for contact frames and wrenches that we measure from the simulation

# contact frame z points along contact normal and (x,y) form a right-handed coosy 
geometry_msgs/TransformStamped contactFrame

# on which Reflex finger is the contact
int32 finger_id

# to which Reflex sensor would this contact belong 
int32 sensor_id

# measured contact wrench in world frame
geometry_msgs/Wrench contact_wrench_world

# magnitudes of contact wrench
float64 contact_force_magnitude
float64 contact_torque_magnitude