#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Planning Challenge
"""

import rospy
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from math import atan2
import numpy as np

class PathPlanning():
    def __init__(self): 
        rospy.init_node("go_straight")
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.speed_msg = Twist()
        self.target_list = rospy.get_param("/points")
        #print(target_list)
        self.cur_pose= [0.0, 0.0, 0.0]
        self.turn_x, self.turn_y, self.turn_z = 0, 0, 0
        self.orient_x, self.orient_y, self.orient_z = 0, 0, 0

    def calc_dist_and_angle(self):
        rospy.loginfo(("current pose before subtraction:", self.cur_pose))
        inc_x = self.target_pose[0] - self.cur_pose[0]
        print("inc_x : ", inc_x)
        inc_y = self.target_pose[1] - self.cur_pose[1]
        print("inc_y : ", inc_y)
        inc_z = self.target_pose[2] - self.cur_pose[2]
        
        self.p2p_distance = (inc_x**2 + inc_y**2 + inc_z**2)**0.5
        
        #turn_x = atan2(inc_z, inc_y)
        #turn_y = atan2(inc_x, inc_z)
        self.turn_z = (atan2(inc_y, inc_x) - self.orient_z) % (2*np.pi)
        
        rospy.loginfo(("Turn ", np.rad2deg(self.turn_z), "deg."))
        rospy.loginfo(("Move ", self.p2p_distance, "m ahead."))
        
        # Update current position
        self.cur_pose = self.target_pose
        rospy.loginfo(("Current pose updated:", self.cur_pose))
        # Update current orientation
        self.orient_z += self.turn_z
        rospy.loginfo(("Current orientation updated:", np.rad2deg(self.orient_z), "deg."))
        
        #return p2p_distance, turn_x, turn_y, turn_z
        #return self.p2p_distance, self.turn_z
        
    def turn_around(self):    
        #theta_x, theta_y, 
        theta_z = 0 #, 0, 0
        
        """t0 = rospy.Time.now().to_sec()
        while (theta_x < turn_x):
            self.speed_msg.angular.x = 0.3
            self.pub.publish(self.speed_msg)
            t1 = rospy.Time.now().to_sec()
            theta_x = self.speed_msg.angular.x * (t1-t0)
        self.speed_msg.angular.x = 0.0
        self.pub.publish(speed_msg)
        #rospy.loginfo("Turned around x-axis.")"""
        
        """t0 = rospy.Time.now().to_sec()
        while (theta_y < turn_y):
            self.speed_msg.angular.y = 0.3
            pub.publish(self.speed_msg)
            t1 = rospy.Time.now().to_sec()
            theta_y = speed_msg.angular.y * (t1-t0)
        speed_msg.angular.y = 0.0
        pub.publish(speed_msg)
        #rospy.loginfo("Turned around y-axis.")"""
        
        t0 = rospy.Time.now().to_sec()
        while (theta_z < self.turn_z):
            self.speed_msg.angular.z = 0.3
            self.pub.publish(self.speed_msg)
            t1 = rospy.Time.now().to_sec()
            theta_z = self.speed_msg.angular.z * (t1-t0)
        self.speed_msg.angular.z = 0.0
        self.pub.publish(self.speed_msg)
        rospy.loginfo("Turned around z-axis.")
        rospy.sleep(1)
        
    def go_straight(self):
        self.speed_msg.linear.x = 0.25
        displacement = 0.0
        t0 = rospy.Time.now().to_sec()
        while (displacement < self.p2p_distance):
            self.pub.publish(self.speed_msg)
            t1 = rospy.Time.now().to_sec()
            displacement = self.speed_msg.linear.x * (t1-t0)
            #print("displacement:", displacement)
        self.speed_msg.linear.x = 0.0
        self.pub.publish(self.speed_msg)
        rospy.loginfo("Target reached!")
        rospy.sleep(1)
    
    def execute(self):
        for next_target in self.target_list:
            #while not rospy.is_shutdown():
            self.target_pose = next_target
            self.calc_dist_and_angle()
            #self.turn_around(turn_x, turn_y, turn_z)
            self.turn_around()
            self.go_straight()
            print("----"*10)

object = PathPlanning()
object.execute()
rospy.loginfo("Main target reached !")
rospy.is_shutdown()