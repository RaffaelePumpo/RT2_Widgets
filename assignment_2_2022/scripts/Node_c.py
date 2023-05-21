#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from tf import transformations
from std_srvs.srv import *
import time
import sys
import select
from assignment_2_2022.msg import Info_rob

freq = 1

def distance(x,des_pos_x,y,des_pos_y):
    ## Compute the distance between the robot and the desired position ##
	dist = ((x-des_pos_x)**2 + (y-des_pos_y)**2)**0.5
	return dist

def average(vel_x,vel_y):
    ## Compute the average speed ##

	average = (vel_x**2 + vel_y**2)**0.5
	return average
	
	
def callback(msg):
	## Callback function ##

    # Get the desired position
    des_pos_x = rospy.get_param("des_pos_x")
    des_pos_y = rospy.get_param("des_pos_y")
        
    # Get the actual position and speed from the custom message
    x = msg.x
    y = msg.y
    vel_x = msg.vel_x
    vel_y = msg.vel_y
        
    # Compute the distance from the goal
    dist = distance(x,des_pos_x,y,des_pos_y)
    
    # Compute the average speed
    avg = average(vel_x,vel_y)
    
    # Get frequency parameter
    Frequency = rospy.get_param("/set_frequency")
    r = rospy.Rate(Frequency)
    
    print("Distance from the goal: " , dist)
    print("Average speed: ", avg)
    
    # Sleep   
    r.sleep()
    
    

def main():

    # Initializes a  node
    rospy.init_node('node_c', anonymous=True)
    
    # Subscriber 
    sub = rospy.Subscriber('/info_rob', Info_rob, callback)
	
    # Wait
    rospy.spin()
    
    
	
if __name__ == "__main__":
	main()	
