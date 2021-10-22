#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time 


def control(linear_speed, turn_rate): 
    rospy.init_node("motor_node")

    """TODO: complete motor publishing functionality here"""
	
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rate = rospy.Rate(1000)
    s_time = time.time()
    
    while not rospy.is_shutdown() and (time.time() -  s_time) <= 1/linear_speed:
        
        twist = Twist()
        twist.linear.x = linear_speed
        
        cmd_pub.publish(twist)

        # rospy.loginfo(twist)
        rate.sleep()

    s_time_new = time.time()
    while not rospy.is_shutdown() and (time.time() -  s_time_new) <= 2 * math.pi / turn_rate:

        twist = Twist()
        twist.angular.z = turn_rate
        
        cmd_pub.publish(twist)

        # rospy.loginfo(twist)
        rate.sleep()
        
    twist = Twist()
    cmd_pub.publish(twist)


if __name__ == "__main__":
    control(0.2, 0.5)
