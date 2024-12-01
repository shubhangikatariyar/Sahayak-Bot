#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import random
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
PI = 3.1415926535897   #Value of pi

#TEAM ID: SahayakBot_2506
#TASK 0 for E-yantra ROBOTICS COMPETITION 2020-21
class turtlebot():

    def __init__(self):
        #Creating our node, publisher= /turtle1/cmd_vel, Subscriber = /turtle1/pose.
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        


    def get_distance(self, r, current_angle):
        #Using formula we know that, Angle= length of arc/ radius of the arc
        distance = r*current_angle
        print (distance)      
        return distance

    def circle_goal(self):
        angle=360
        goal_pose = Pose()
        goal_pose.x = random.randint(2, 2)
        goal_pose.y = random.randint(2, 2)
        distance_tolerance = 0.1
        vel_msg = Twist()
        angular_speed = 30*2*PI/360
        relative_angle = angle*2*PI/360
        

        #To get VAlue of current time for calculation of distance
        t0 = rospy.Time.now().to_sec()
        current_angle = 0 
        print("Move")
        
        while(current_angle <= relative_angle):
       

            #Value for linear velocity in x-axis:
            vel_msg.linear.x = 0.8
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #Value for angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0.5
            
            #instantaneous time at t sec
            t1 = rospy.Time.now().to_sec()
            #FORMULA
            current_angle = angular_speed*(t1-t0)
	   

           
            self.velocity_publisher.publish(vel_msg)  #FOR REVOLVE
            self.rate.sleep()
            time.sleep(1)
           

            rospy.loginfo("Moving in a circle")
            self.get_distance(goal_pose.x,current_angle)
            

        # Stopping our robot after the movement is over and terminating the while loop
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("goal reached")    #message after completion of revolution



        

if __name__ == '__main__':
    try:
        x = turtlebot()     #calling the class turtlebot()
        x.circle_goal()
    except rospy.ROSInterruptException:
        pass
