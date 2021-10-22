#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


lin_spd = 0.1
ang_spd = 0.1
twist = None
cmd_pub = None

def fwrd(x):
	ft = 5.4 * (float(x) / 50)
	rate = rospy.Rate(1000)
	while not rospy.is_shutdown() and ft > 0:
		twist.linear.x = lin_spd
		twist.angular.z = 0
		cmd_pub.publish(twist)
		ft -= 0.001
		rate.sleep()
def ang(x):
        at = 66/8 * (x / 45)
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown() and at > 0:
                twist.linear.x = 0
                twist.angular.z = ang_spd
                cmd_pub.publish(twist)
                at -= 0.001
                rate.sleep()
def part3(x, z):
	print('part3')
	v = 0.05
	w = 0.1 *z
	at = 8 * (float(x) / 45)
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown() and at > 0:
                print(at)
		twist.linear.x = v
                twist.angular.z = w
                cmd_pub.publish(twist)
                at -= 0.001
                rate.sleep()
	


def publisher_node():
	global cmd_pub, twist
	cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
	twist = Twist()
	#4.1::
	#fwrd(200)
	#ang(90)
	#fwrd(50)
	#ang(45)

	#4.2:
	
	#fwrd(100)
	#ang(90)
	#fwrd(100)
	#ang(90)
	#fwrd(100)
	#ang(90)
	#fwrd(100)

	#4.3:
	'''
	fwrd(183)
	
	twist.linear.x = 0
        twist.angular.z = 0
        cmd_pub.publish(twist)
	part3(135)
	'''

	#4.4:

	t = 0
	sqrt = math.sqrt
	pi = math.pi
	rate = rospy.Rate(1000)
	b = float(15)
	while not rospy.is_shutdown():
		
		v = sqrt(4*pi*pi*(math.cos(pi / b * t)**2) + 1) / 5
		# print(v)
		twist.linear.x = v
		w  = (-1*2*pi*pi / b * math.sin(pi/b*t)) / v**3
		print(w)
		twist.angular.z = w
		cmd_pub.publish(twist)
		t += 0.001
		rate.sleep()

	'''
	for i in range(4):
		part3(90, 1)
		part3(90, -1)
	'''

	twist.linear.x = 0
	twist.angular.z = 0
	cmd_pub.publish(twist)
	print("done")
	#rospy.spin()

def main():

    try:
        rospy.init_node('lab02')
        publisher_node()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
