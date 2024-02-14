#!/usr/bin/env python3

import sys
import rospy
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String

sys.path.insert(0, '/ros-lab/interbotix_ws/')

class InterbotixXsApiTester:
    def __init__(self):
        self.xs_api = InterbotixManipulatorXS("px100", "arm", "gripper")
        
    def draw_square_and_move_gripper(self):
        self.xs_api.arm.go_to_home_pose()
        self.xs_api.arm.set_ee_cartesian_trajectory(z=-0.1)
        self.xs_api.arm.set_ee_cartesian_trajectory(x=-0.1)
        self.xs_api.arm.set_ee_cartesian_trajectory(z=0.1)
        self.xs_api.arm.set_ee_cartesian_trajectory(x=0.1)
        self.xs_api.gripper.open()
        self.xs_api.gripper.close()
        self.xs_api.arm.go_to_sleep_pose()

class ArmControlApiTester:
    def __init__(self):
        self.home_publisher = rospy.Publisher("/arm_control/home", Bool, queue_size=1)
        self.sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
        self.gripper_publisher = rospy.Publisher("/arm_control/gripper", String, queue_size=1)
        # It's important to set the queue_size of the point_publisher 
        # to a large enough value so no published messages get lost.
        # `4` seems to work for the draw_square() method.
        self.point_publisher = rospy.Publisher("/arm_control/point", Point, queue_size=10)

    def draw_square(self):
        self.point_publisher.publish(Point(0, 0, -0.1))
        self.point_publisher.publish(Point(-0.1, 0, 0))
        self.point_publisher.publish(Point(0, 0, 0.1))
        self.point_publisher.publish(Point(0.1, 0, 0))

    def draw_square_at_waist_position(self, y_target):
        self.point_publisher.publish(Point(0, y_target, -0.1))
        self.point_publisher.publish(Point(-0.1, y_target, 0))
        self.point_publisher.publish(Point(0, y_target, 0.1))
        self.point_publisher.publish(Point(0.1, y_target, 0))

    def draw_square_at_two_different_waist_positions(self):
        self.point_publisher.publish(Point(0, -1, 0))
        self.draw_square_at_waist_position(-1)
        self.point_publisher.publish(Point(0, 1, 0))
        self.draw_square_at_waist_position(1)

if __name__=='__main__':

    # Interbotix API Testing

    tester = InterbotixXsApiTester()
    tester.draw_square_and_move_gripper()

    # Arm Control Testing
    # rospy.init_node("arm_control_api_tester")
    
    # tester = ArmControlApiTester()

    # first_message_published = False;

    # rate = rospy.Rate(0.4)

    # while not rospy.is_shutdown():
    #     rate.sleep()

