import rospy
from std_srvs.srv import Empty


class GazeboInterface:
    def __init__(self):

        self.unpause_name = "/gazebo/unpause_physics"
        self.pause_name = "/gazebo/pause_physics"
        self.unpause = rospy.ServiceProxy(self.unpause_name, Empty)
        self.pause = rospy.ServiceProxy(self.pause_name, Empty)

    def sim_unpause(self):
        rospy.wait_for_service(self.unpause_name)
        try:
            self.unpause()
        except rospy.ServiceException as e:
            rospy.loginfo(self.unpause_name + " service call failed")

    def sim_pause(self):
        rospy.wait_for_service(self.pause_name)
        try:
            self.pause()
        except rospy.ServiceException as e:
            rospy.loginfo(self.pause_name + " service call failed")

    def run_for_seconds(self, prefix, secs, cmd_str):
        self.sim_unpause()
        rospy.loginfo(f"{prefix}: Executing " + cmd_str + f" for {secs} secs.")
        rospy.sleep(secs)
        self.sim_pause()

    def get_dist_tcp_obj(self):
        pass
