#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def demo_ride():

    rospy.init_node("demo_movement")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Demo running")
    pause = 5
    vel = Twist()
    
    # Move forwards
    rospy.sleep(pause)
    vel.linear.x = 0.218
    vel.angular.z = 0.895
    pub.publish(vel)
    rospy.sleep(pause)
    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)
    



if __name__ == "__main__":
    demo_ride()
