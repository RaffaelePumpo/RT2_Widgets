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



def callback(msg):
## Function to get the position and velocity from the message and publish it  to the custom message ##

    global pub
 
 # Get the position and velocity from the message
    pos = msg.pose.pose.position
    vel = msg.twist.twist.linear
    
    # Create the custom message
    info_rob = Info_rob()
    info_rob.x = pos.x
    info_rob.y = pos.y
    info_rob.vel_x = vel.x
    info_rob.vel_y = vel.y
    
    # Publish it
    pub.publish(info_rob)
        
        
        
def client_function():
    ## Function to create the client and send the goal to the server or cancel it ##

    # Create the action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)

    # Waits the server to be ready
    client.wait_for_server()
    
	
    while not rospy.is_shutdown():
        
        print("Insert the x and y position of the target or c to cancel")

        # Get the position from the user
        pos_x = input("x position or c to cancel: ")
        pos_y = input("y position or c to cancel: ")
        
        # Cancel the goal if the user insert c
        if pos_x == "c" or pos_y=="c":
            
            # Cancel the goal
            client.cancel_goal()


        else:
            # Convert the position from string to float
            x = float(pos_x)
            y = float(pos_y)
            
            # Create the goal
            goal = assignment_2_2022.msg.PlanningGoal()

            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
					
            # Send it to the server
            client.send_goal(goal)
            


       
def main():

    global pub
    
    try:
        # Initializes a rospy node 
        rospy.init_node('client_node_a')
        
        # Publisher to publish the custom message
        pub = rospy.Publisher("/info_rob", Info_rob, queue_size=10)
        
        # Subscriber 
        sub = rospy.Subscriber('/odom', Odometry, callback)
        
        # Start client
        client_function()
        
               
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
        
        
   
if __name__ == '__main__':
    main()
