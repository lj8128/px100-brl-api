#!/usr/bin/env python3

import sys
import rospy
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32
from queue import Queue

# if your paths are messed up and not recognizing the interbotix_ws, 
# then you can use this to get the directory working with this file
sys.path.insert(0, '/ros-lab/interbotix_ws/') #THIS PATH WORKS ON TKINKPAD TWO
# sys.path.insert(0, '/my_ros_data/interbotix_ws/') #THIS PATH WORKS ON THE OLDER THINKPAD


class ArmControl():

    def __init__(self):
        # get the robot arm connection set up
        self.bot = InterbotixManipulatorXS("px100", "arm", "gripper")
        self.bot.arm.set_trajectory_time(1)

        # set up the Subscribers 
        self.point_sub = rospy.Subscriber("/arm_control/point", Point, self.point_cb)
        self.pose_sub = rospy.Subscriber("/arm_control/pose", Point, self.pose_cb)
        self.home_sub = rospy.Subscriber("/arm_control/home", Bool, self.home_cb)
        self.sleep_sub = rospy.Subscriber("arm_control/sleep", Bool, self.sleep_cb)
        self.gripper_sub = rospy.Subscriber("/arm_control/gripper", String, self.gripper_cb)
        self.exit_sub = rospy.Subscriber("/arm_control/exit", Bool, self.exit_cb)
        self.time_sub = rospy.Subscriber("/arm/time", Float32, self.time_cb)

        # initialize the coordinates and booleans
        self.prev_x = 0
        self.prev_y = 0
        self.prev_z = 0
        self.pose = False
        self.home = False 
        self.sleep = False
        self.exit = False
        self.neutral = "NA"
        self.gripper = self.neutral
        self.open = "open"
        self.close = "close"
        self.time = 0 

        # initialize Queue for buffering target coordinates for
        # command_point
        self.queue = Queue()
        
        # set initial ee pose components 
        self.pose_x = 0.2
        self.pose_y = 0
        self.pose_z = 0.1
        rospy.loginfo("set_ee_pose_components: ({},{},{})".format(self.pose_x, self.pose_y, self.pose_z))
        self.bot.arm.set_ee_pose_components(x=self.pose_x, y=self.pose_y, z=self.pose_z)


        self.main()


    # point message callback - set the x,y,z coordinates 
    def point_cb(self, msg):
        self.queue.put(msg) 
    
    # ee pose components call back - set a new ee pose component with x, y, z coordinates
    def pose_cb(self, msg):
        self.pose = True
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_z = msg.z

        rospy.loginfo("Change the ee pose components to: ({},{},{})".format(self.pose_x, self.pose_y, self.pose_z))
    
    # home callback - True if the arm is commanded to go to the home position
    def home_cb(self, msg):
        self.home = msg
    
    # sleep callback - True if the arm is commanded to go to the sleep position
    def sleep_cb(self, msg):
        self.sleep = msg
    
    # gripper callback - get string message "open" or "close"
    def gripper_cb(self, msg):
        self.gripper = msg.data
    
    def exit_cb(self, msg):
        self.exit = msg
    
    def time_cb(self, msg):
        self.time = msg.data

    # command the arm to go to the received coordinates 
    def command_point(self):
        target = self.queue.get()
        rospy.loginfo("Target coordinate {}".format(target))
        # since the PX100 has 4dof, the y or yaw cannot be specified - instead we change the waist position
        # to change the direction the arm is facing 
        # this means that the y coordinate is processed differently than the x and y coordinates 
        # the y coordinate is an absolute position which means the 0.3 will always return the same position 
        if target.y != self.prev_y: 
            if(target.y<-2.7): #Set min and max rotation to avoid hitting the pole
                rospy.loginfo("moving waist to {}".format(-2.7))
                self.bot.arm.set_single_joint_position("waist", -2.7)
            elif target.y>2.7:
                rospy.loginfo("moving waist to {}".format(2.7))
                self.bot.arm.set_single_joint_position("waist", 2.7)
            else:
                rospy.loginfo("moving waist to {}".format(target.y))
                self.bot.arm.set_single_joint_position("waist", target.y)
        # x and z are relative coordinates. If the initial x position of the arm is 0.1, then if the command received is 0.1
        # the new x position will be 0.2. Same with z.
        if target.x or target.z != 0: 
            rospy.loginfo("move ee by x={}, z={}".format(target.x, target.z))
            self.bot.arm.set_ee_cartesian_trajectory(x=target.x, z=target.z)
        self.prev_x = target.x
        self.prev_y = target.y 
        self.prev_z = target.z
    
    # set a new ee pose component
    def command_pose(self):
        rospy.loginfo("set_ee_pose_components: ({},{},{})".format(self.pose_x, self.pose_y, self.pose_z))
        self.bot.arm.set_ee_pose_components(x=self.pose_x, y=self.pose_y, z=self.pose_z)

    # command gripper to open or close based on incoming command
    def command_gripper(self):
        # print(self.gripper)
        if self.gripper == self.open: 
            rospy.loginfo("Opening the gripper")
            self.bot.gripper.open()
        elif self.gripper == self.close: 
            rospy.loginfo("Closing the gripper")
            self.bot.gripper.close()

    def target_points_exist(self):
        return not self.queue.empty()

    # logic to process all the commands received until the program is quit out of
    def main(self):
        while not rospy.is_shutdown():
            if self.target_points_exist():
                self.command_point()
            elif self.pose: 
                self.command_pose()
                self.pose = False 
            elif self.home: 
                rospy.loginfo("Going to home position")
                self.bot.arm.go_to_home_pose()
                self.home = False
            elif self.gripper != self.neutral:
                self.command_gripper()
                self.gripper = self.neutral
            elif self.sleep: 
                rospy.loginfo("Going to sleep position")
                self.bot.arm.go_to_sleep_pose()
                self.sleep = False 
            elif self.time != 0: 
                rospy.loginfo("Setting trajectory time to {}".format(self.time))
                self.bot.arm.set_trajectory_time(self.time)
                self.time = 0
            elif self.exit: 
                rospy.loginfo("The send_command node is exiting... going to my sleep pose")
                self.bot.arm.go_to_sleep_pose()
                self.exit = False 

                

if __name__=='__main__':
    try:
        ArmControl()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
