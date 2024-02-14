#!/usr/bin/env python3
"""
A basic ROS command line controller for the Interbotix PX100 Arm. 
Run the program, read the README or code, to find more details about the commands
"""

import sys
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import rospy, math
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32

sys.path.insert(0, '/ros-lab/interbotix_ws/')

class SendCommand():
    def __init__(self):
        self.xs_api = InterbotixManipulatorXS("px100", "arm", "gripper")
        
        # set initial pose of robot.
        self.xs_api.arm.set_ee_pose_components(x=0.2, z=0.1)
        rospy.loginfo("set_ee_pose_components: ({},{},{})".format(0.2, 0, 0.1))

        # Coordinates to be sent to the arm
        self.x = 0 
        self.y = 0
        self.z = 0

        self.arm_x=0
        self.arm_y=0

        self.current_x=0
        self.current_y=0
        self.current_z=0

        self.at_height=False

        self.old_box_x=0
        self.old_box_y=0

        self.state="waiting"

        # Cargo's physical coordinates
        self.cargo_point_sub = rospy.Subscriber("cargo_point", Point, self.cargo_point_cb)
        self.arm_point_sub=rospy.Subscriber("arm_point", Point, self.arm_point_cb)

        # Coordinate with the transportation robot
        self.alien_state_sub = rospy.Subscriber("alien_state", Bool, self.set_state)
        self.alien_state = False

        self.arm_status_publisher = rospy.Publisher("/arm_status", String, queue_size=1)

    # True if robot has finished moving to loading zone and False otherwise
    def set_state(self, msg):
        self.alien_state = msg.data

    # Drop cargo to preset destination zone
    def drop_cargo(self):
        self.xs_api.arm.set_single_joint_position("waist", -1.2)
        self.xs_api.arm.set_ee_cartesian_trajectory(z=-0.08, moving_time=0.3)
        self.xs_api.arm.set_ee_cartesian_trajectory(x=-0.08, moving_time=0.3)
        self.xs_api.gripper.open() 
        self.xs_api.arm.set_ee_cartesian_trajectory(x=0.08, moving_time=0.3)
        self.xs_api.arm.set_ee_cartesian_trajectory(z=0.1, moving_time=0.3)
        self.xs_api.arm.go_to_home_pose()
        self.xs_api.arm.go_to_sleep_pose()
        self.xs_api.gripper.close()
        self.arm_status_publisher.publish("resting")

    def arm_point_cb(self,msg):
        self.arm_x=msg.x
        self.arm_y=msg.y

    # Calculate the location of the Cargo based on the sent location.
    def cargo_point_cb(self, msg):
        if(self.at_height==False and self.alien_state==True and self.state=="waiting"):
            self.state="setting_height"
            if(self.old_box_x==0):
                self.old_box_x=msg.x
                self.old_box_y=msg.y

            self.xs_api.arm.go_to_home_pose()
            self.xs_api.gripper.open()
            self.xs_api.arm.set_ee_cartesian_trajectory(z=-0.1, moving_time=0.4)
            self.xs_api.arm.set_ee_cartesian_trajectory(x=-0.05, moving_time=0.4)

            self.at_height=True
            self.state="moving"

        if (self.alien_state and self.state=="moving"):	# Calculate only if robot is in position
            if(self.arm_y!=0 and self.arm_x!=0):
                flags=0
                # distance=math.sqrt( ((self.arm_x-self.old_box_x)**2)+((self.arm_y-self.old_box_x)**2))
                # print("Distance: "+str(distance))
                # print("X: "+str(self.old_box_x-self.arm_x))
                # print("Y: "+str(y-self.arm_y))
                if (abs(self.old_box_x-self.arm_x)<=6):
                    flags+=1
                elif(self.old_box_x>self.arm_x):
                    self.current_y-=0.03
                elif(self.old_box_x<self.arm_x):
                    self.current_y+=0.03

                if (abs((self.old_box_y)-self.arm_y)<=10):
                    self.current_x=0
                    flags+=1
                elif(self.old_box_y>self.arm_y):
                    self.current_x-=0.01
                elif(self.old_box_y<self.arm_y):
                    self.current_x+=0.01

                if(flags==2):
                    self.state="descending"
                else:
                    self.xs_api.arm.set_single_joint_position("waist", self.current_y, moving_time=0.05, accel_time=0)
                    self.xs_api.arm.set_ee_cartesian_trajectory(x=self.current_x, moving_time=0.3)
                    self.current_x=0

        elif self.state=="descending" and self.alien_state:
            if(abs(self.current_y>0.2)):
                if(self.current_y<0):
                    self.current_y+=0.03
                else:
                    self.current_y-=0.03

            self.xs_api.arm.set_single_joint_position("waist", self.current_y, moving_time=0.05, accel_time=0)
            self.xs_api.arm.set_ee_cartesian_trajectory(z=-0.01, moving_time=0.8)
            self.xs_api.arm.set_ee_cartesian_trajectory(z=-0.03, moving_time=0.8)

            self.xs_api.gripper.close()

            # a side effect of feeding the ee_cartesian_trajectory
            # method a moving_time argument `t` is that it sets the
            # moving time for all subsequent trajectories to `t`. In
            # our case, this makes the robot go to the sleep pose a bit
            # too violently. So we set the moving_time to 1 via the
            # set_trajectory_time method.
            self.xs_api.arm.set_trajectory_time(moving_time=1)
            self.xs_api.arm.go_to_sleep_pose()
            self.alien_state=False
            self.at_height=False
            self.current_y=0
            self.alien_state = False 

            self.old_box_x=0
            self.old_box_y=0
            self.state="waiting"

commander = SendCommand()
rospy.spin()
