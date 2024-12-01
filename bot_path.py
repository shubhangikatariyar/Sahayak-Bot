#! /usr/bin/env python

# Import Statements
import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion
import numpy as np
import time
import math


pose = [0,0,0]
regions = { 'bright':2 ,
        'fright':2 ,
        'front': 2,    
        'fleft':2 , 
        'bleft': 2 }  

def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w  
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]] 

def laser_callback(msg):
    global regions
    regions = {
        'bright': min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front': min(min(msg.ranges[288:431]), 10),   
        'fleft': min(min(msg.ranges[432:575]), 10),
        'bleft': min(min(msg.ranges[576:719]), 10),
    }

# Function to rotate Left
def L_rotate(velocity_msg,pub,angle):  
    global pose
    ia1= pose[2] 
    ia2 = pose[2] + angle  
    while ia1 < ia2:   
        velocity_msg.linear.x = 0
        pub.publish(velocity_msg)
        velocity_msg.angular.z = 1     
        pub.publish(velocity_msg)
        ia1= pose[2]   #instant 
    velocity_msg.linear.x = 0
    pub.publish(velocity_msg)
    velocity_msg.angular.z = 0      
    pub.publish(velocity_msg)
    return

# Function to rotate right
def R_rotate(velocity_msg,pub,angle):
    global pose
    ia1= pose[2] #current angle
    ia2 = pose[2] - angle  #desired theta 
    while ia1 > ia2:   
        velocity_msg.linear.x = 0
        pub.publish(velocity_msg)
        velocity_msg.angular.z = -1      
        pub.publish(velocity_msg)
        ia1= pose[2]   #instant 
    velocity_msg.linear.x = 0
    pub.publish(velocity_msg)
    velocity_msg.angular.z = 0      
    pub.publish(velocity_msg)
    return

# Function to align the robot on the direction of the goal
def go_to_goal(velocity_msg,pub):
    x,y = 12.5 , 0  #GOAL
    desired_angle_goal=math.atan2(y-pose[1], x-pose[0])
    bot_angle= pose[2]
    if bot_angle > desired_angle_goal :
        diff_angle = bot_angle-desired_angle_goal
        R_rotate(velocity_msg,pub,diff_angle)
        
    else :
        diff_angle = desired_angle_goal- bot_angle
        L_rotate(velocity_msg,pub,diff_angle)
    
# Main Function deciding the step taken by the bot 
def logic(velocity_msg,pub):
    
    global pose, regions
    x,y = 0,0
    temp = False
    while x <= 6.28:
        y= 2*(math.sin(x))*(math.sin(x/2))   
        K_angular = 2.5  #1.95
        desired_angle_goal=math.atan2(y-pose[1], x-pose[0])
        velocity_msg.angular.z=(desired_angle_goal-pose[2])*K_angular
        pub.publish(velocity_msg)
        distance = math.sqrt((x-pose[0])**2+(y-pose[1])**2)
        velocity_msg.linear.x = distance*(0.4)  #0.8
        pub.publish(velocity_msg)
        rospy.sleep(1.)
        x = x + 0.30925

    while True : 
        if pose[0] >= 12.2 and pose[0] < 12.8 and pose[1] > -0.3 and pose[1] <0.3:
            break
        if regions['front'] > 4 and regions['fleft']> 4 and regions['fright']> 4 and temp :
            velocity_msg.linear.x= 1  #1
            pub.publish(velocity_msg)
            rospy.sleep(0.4)
            velocity_msg.linear.x= 0
            pub.publish(velocity_msg)
            go_to_goal(velocity_msg,pub)
            temp = False
        

        if regions['front'] > 2 : 
            velocity_msg.linear.x= 1
            pub.publish(velocity_msg)
            rospy.sleep(0.5) #0.65
            velocity_msg.linear.x= 0
            pub.publish(velocity_msg)

        else: 
            temp = True
            if regions['fleft'] > 5 :  #0.15
                L_rotate(velocity_msg,pub,math.pi/18)
            else:
                R_rotate(velocity_msg,pub,math.pi/18)
    velocity_msg.linear.x= 0
    pub.publish(velocity_msg)
    velocity_msg.angular.z= 0
    pub.publish(velocity_msg)

def control_loop():
    
    global pose
    rospy.init_node('ebot_controller')
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    velocity_msg=Twist()
    cmd_vel_topic='/cmd_vel'
   
    while True : 
            logic(velocity_msg,pub)
            break

if __name__ == '__main__':
    try:
        control_loop()    #call main here 
    except rospy.ROSInterruptException:
        pass