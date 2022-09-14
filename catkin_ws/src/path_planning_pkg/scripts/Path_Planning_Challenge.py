#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Planning Challenge
"""

import rospy
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry

# class GotoTarget():
    #def __init__(self):
 
rospy.init_node("go_straight")
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
speed_msg = Twist()

"""for get_target_list(i):
    return rospy.get_param("/points")[i]"""
target_list = rospy.get_param("/points")
print(target_list)
cur_pose= [0.0, 0.0, 0.0]

def calc_dist_and_angle(target_pose,cur_pose):
    inc_x = target_pose[0] - cur_pose[0]
    inc_y = target_pose[1] - cur_pose[1]
    inc_z = target_pose[2] - cur_pose[2]
    p2p_distance = (inc_x**2+inc_y**2+inc_z**2)**0.5
    
    turn_x = atan2(inc_z, inc_y)
    turn_y = atan2(inc_x, inc_z)
    turn_z = atan2(inc_y, inc_x)
    return p2p_distance, turn_x, turn_y, turn_z
    
def turn_around(turn_x, turn_y, turn_z):    
    theta_x, theta_y, theta_z = 0, 0, 0
    
    t0 = rospy.Time.now().to_sec()
    while (theta_x < turn_x):
        speed_msg.angular.x = 0.3
        pub.publish(speed_msg)
        t1 = rospy.Time.now().to_sec()
        theta_x = speed_msg.angular.x * (t1-t0)
    speed_msg.angular.x = 0.0
    pub.publish(speed_msg)
    rospy.loginfo("Turned around x-axis.")
    
    t0 = rospy.Time.now().to_sec()
    while (theta_y < turn_y):
        speed_msg.angular.y = 0.3
        pub.publish(speed_msg)
        t1 = rospy.Time.now().to_sec()
        theta_y = speed_msg.angular.y * (t1-t0)
    speed_msg.angular.y = 0.0
    pub.publish(speed_msg)
    rospy.loginfo("Turned around y-axis.")
    
    t0 = rospy.Time.now().to_sec()
    while (theta_z < turn_z):
        speed_msg.angular.z = 0.3
        pub.publish(speed_msg)
        t1 = rospy.Time.now().to_sec()
        theta_z = speed_msg.angular.z * (t1-t0)
    speed_msg.angular.z = 0.0
    pub.publish(speed_msg)
    rospy.loginfo("Turned around z-axis.")
    
def go_straight(p2p_distance):
    speed_msg.linear.x = 0.05
    displacement = 0.0
    t0 = rospy.Time.now().to_sec()
    while (displacement < p2p_distance):
        pub.publish(speed_msg)
        t1 = rospy.Time.now().to_sec()
        displacement = speed_msg.linear.x * (t1-t0)
    speed_msg.linear.x = 0.0
    pub.publish(speed_msg)
    rospy.loginfo("Target reached!")

def execute():
    for next_target in target_list:
        target_pose = next_target
        p2p_distance, turn_x, turn_y, turn_z = calc_dist_and_angle(target_pose, cur_pose)
        turn_around(turn_x, turn_y, turn_z)
        go_straight(p2p_distance)
        
execute()
rospy.loginfo("Main target reached !")
rospy.is_shutdown()
