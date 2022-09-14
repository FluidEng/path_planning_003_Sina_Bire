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
        self.cur_pose= [0.0, 0.0, 0.0]
        self.cur_orient = [0.0, 0.0, 0.0]
        #self.turn_x, self.turn_y, self.turn_z = 0.0, 0.0, 0.0
        self.rotation = [0.0, 0.0, 0.0]

    def calc_dist_and_angle(self):
        print("current pose before subtraction:", self.cur_pose)
        inc_vector=[0, 0, 0]
        
        # Calculate current position to next target distance
        for i in range(3):
            inc_vector[i] = self.target_pose[i] - self.cur_pose[i]
        print("inc_vector: ", inc_vector)
        print("cur_orient: ", np.rad2deg(self.cur_orient))
        self.p2p_distance  = (sum([num ** 2 for num in inc_vector]))**0.5
        
        """self.rotate_z = (atan2(inc_vector[1],inc_vector[0]) - self.cur_orient[2]) % (2*np.pi)
        if self.rotate_z > np.pi:
            self.rotate_z -= (2*np.pi)"""
            
        # Calculate rotation angles
        for i in range(3):
            # rotate_x = (atan2(inc_z, inc_y) - cur_orient_x)
            # rotate_z = (atan2(inc_y, inc_z) - cur_orient_z)
            if (abs(inc_vector[(i+1)%3]) < 1e-3) or (abs(inc_vector[(i+2)%3])< 1e-3) :
                self.rotation[i] = 0.0
            else:
                print("atan2 = ", np.rad2deg(atan2(inc_vector[(i+2)%3], (inc_vector[(i+1)%3]))))
                self.rotation[i] = atan2(inc_vector[(i+2)%3], (inc_vector[(i+1)%3])) - self.cur_orient[i]
            print("inc_vector_nom =", inc_vector[(i+2)%3])
            print("inc_vector_den =", inc_vector[(i+1)%3])
            #print("arctan: ", np.rad2deg( np.arctan(inc_vector[(i+2)%3]/ (inc_vector[(i+1)%3])) % (2*np.pi) ))
            print("cur_orient",i,":", np.rad2deg(self.cur_orient[i]))
            print("difference:", np.rad2deg(self.rotation[i]))
            print("---")
            
            # Take mode 360deg of rotation angle
            self.rotation[i] = self.rotation[i] % (2*np.pi)
            # Decide left/right turn
            if self.rotation[i] > np.pi:
                self.rotation[i] = self.rotation[i] - (2*np.pi)
        """ 0 2 1
            1 0 2
            2 1 0"""
        print("Rotation b/w current pose and next pose: ", np.rad2deg(self.rotation)," deg")
        print("Move ", self.p2p_distance, "m ahead.")
        
        
        
        """if abs(inc_vector[2]) < 0.001:
            self.rotate_x = 0
        else:
            self.rotate_x = (atan2(inc_z, inc_y) - self.cur_orient[0]) % (2*np.pi)
        
        if abs(inc_x) < 0.001:
            self.rotate_y = 0
        else:
            self.rotate_y = (atan2(inc_x, inc_z) - self.cur_orient[1]) % (2*np.pi)
    
        if abs(inc_y) < 0.001:
            self.rotate_z = 0
        else:    
            self.rotate_z = (atan2(inc_y, inc_x) - self.cur_orient[2]) % (2*np.pi)
        self.rotation = [self.rotate_x, self.rotate_y, self.rotate_z]
        """

        
        # Update current position
        self.cur_pose = self.target_pose
        #print(("Current pose updated:", self.cur_pose))
        
        # Update current orientation
        self.cur_orient = np.add(self.cur_orient, self.rotation) % (2*np.pi)
        #print(("Current orientation updated:", np.rad2deg(self.cur_orient), "deg."))
        
        #return p2p_distance, turn_x, turn_y, turn_z
        #return self.p2p_distance, self.turn_z
        
    def turn_around(self):    
        theta_x, theta_y, theta_z = 0.0, 0.0, 0.0
        
        t0 = rospy.Time.now().to_sec()
        while (abs(theta_x) < abs(self.rotation[0])):
            self.speed_msg.angular.x = np.sign(self.rotation[0])*0.3
            self.pub.publish(self.speed_msg)
            t1 = rospy.Time.now().to_sec()
            theta_x = self.speed_msg.angular.x * (t1-t0)
        self.speed_msg.angular.x = 0.0
        self.pub.publish(self.speed_msg)
        #rospy.loginfo("Turned around x-axis.")
        
        t0 = rospy.Time.now().to_sec()
        while (abs(theta_y) < abs(self.rotation[1])):
            self.speed_msg.angular.y = np.sign(self.rotation[1])*0.3
            self.pub.publish(self.speed_msg)
            t1 = rospy.Time.now().to_sec()
            theta_y = self.speed_msg.angular.y * (t1-t0)
        self.speed_msg.angular.y = 0.0
        self.pub.publish(self.speed_msg)
        #rospy.loginfo("Turned around y-axis.")
        
        t0 = rospy.Time.now().to_sec()
        while (abs(theta_z) < abs(self.rotation[2])):
            self.speed_msg.angular.z = np.sign(self.rotation[2])*0.3
            self.pub.publish(self.speed_msg)
            t1 = rospy.Time.now().to_sec()
            theta_z = self.speed_msg.angular.z * (t1-t0)
        self.speed_msg.angular.z = 0.0
        self.pub.publish(self.speed_msg)
        print("Turned around z-axis.")
        #rospy.sleep(1)
        
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
        print("Target reached!")
        rospy.sleep(1)
    
    def execute(self):
        for next_target in self.target_list:
            #while not rospy.is_shutdown():
            self.target_pose = next_target
            self.calc_dist_and_angle()
            self.turn_around()
            self.go_straight()
            print("----"*10)

object = PathPlanning()
object.execute()
print("Main target reached !")
rospy.is_shutdown()