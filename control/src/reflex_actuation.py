#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty, Trigger, TriggerResponse
from std_msgs.msg import Bool, Float64

"""A script to execute high level functions of the Reflex Takktile robotic hand in simulation."""

class ReflexGraspInterface:

    def __init__(self):
        
        # topics 
        self.preshape_1_cmd_topic = "/lbr4_reflex/preshape_1_effort_controller/command"
        self.preshape_2_cmd_topic = "/lbr4_reflex/preshape_2_effort_controller/command"
        self.proximal_1_cmd_topic = "/lbr4_reflex/proximal_joint_1_effort_controller/command"
        self.proximal_2_cmd_topic = "/lbr4_reflex/proximal_joint_2_effort_controller/command"
        self.proximal_3_cmd_topic = "/lbr4_reflex/proximal_joint_3_effort_controller/command"
        
        # commands
        self.proximal_1_cmd = Float64()
        self.proximal_2_cmd = Float64()
        self.proximal_3_cmd = Float64()

        # publishers 
        self.preshape_1_cmd_pub = rospy.Publisher(self.preshape_1_cmd_topic, Float64, queue_size=1)
        self.preshape_2_cmd_pub = rospy.Publisher(self.preshape_2_cmd_topic, Float64, queue_size=1)
        self.proximal_1_cmd_pub = rospy.Publisher(self.proximal_1_cmd_topic, Float64, queue_size=1)
        self.proximal_2_cmd_pub = rospy.Publisher(self.proximal_2_cmd_topic, Float64, queue_size=1)
        self.proximal_3_cmd_pub = rospy.Publisher(self.proximal_3_cmd_topic, Float64, queue_size=1)

        # offered services 
        self.open_hand_srv = rospy.Service("/reflex/open_hand", Trigger, self.open_hand)
        self.close_hand_srv = rospy.Service("/reflex/close_hand", Trigger, self.close_hand)

        self.rate = rospy.Rate(100)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def stop(self):
        rospy.loginfo("Exiting.")

    def command_hand(self, prox_1, prox_2, prox_3):
        self.proximal_1_cmd.data = prox_1
        self.proximal_2_cmd.data = prox_2
        self.proximal_3_cmd.data = prox_3

        for i in range(10):
            self.proximal_1_cmd_pub.publish(self.proximal_1_cmd)
            self.proximal_2_cmd_pub.publish(self.proximal_2_cmd)
            self.proximal_3_cmd_pub.publish(self.proximal_3_cmd)

    def close_hand(self, req):
        self.command_hand(0.5, 0.5, 0.5)
        return TriggerResponse(success=True)

    def open_hand(self, req):
        self.command_hand(-0.5, -0.5, -0.5)
        return TriggerResponse(success=True)


if __name__ == '__main__':
    rospy.init_node("reflex_actuation")
    try:
        interface = ReflexGraspInterface()
        rospy.on_shutdown(interface.stop)
        interface.run()
    except rospy.ROSInterruptException:
        pass