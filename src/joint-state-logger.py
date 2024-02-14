#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

class JointStateLogger:
    def __init__(self):
       self.joint_states_sub = rospy.Subscriber("/px100/joint_states", JointState, self.log_joint_states)

    def log_joint_states(self, msg):
        rospy.loginfo(msg)

if __name__=='__main__':
    try:
        rospy.init_node("joint_state_logger")
        JointStateLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

