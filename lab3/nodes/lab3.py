#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, os
import numpy as np
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

def getKey(): #you can ignore this function. It's for stopping the robot when press 'Ctrl+C'
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

STRAIGHT = 0
TURNING = 1

class PIDcontrol():
    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.color_sub = rospy.Subscriber('line_idx', String, self.camera_callback, queue_size=1)
        self.twist = Twist()
        self.actual = 320
        self.desired = 320
        self.v = 0.1
        self.k = 0.2
        self.kp = 0.004  # 0.025 -> perfect oscillation
        self.ki = 0.00001
        self.kd = 0.001
        self.integral = 0
        self.maxIntegral = 1000
        self.lasterror = 0
        self.rate = rospy.Rate(15)
        self.log = open('log.txt', 'w')
        self.state = STRAIGHT



    def camera_callback(self, data):
        self.actual = int(data.data)
        # print(self.actual)


    def follow_the_line(self):
        self.race()
        # self.test()
        self.cmd_pub.publish(self.twist)
        self.rate.sleep()


    def test(self):
        self.twist.linear.x = self.v
        self.twist.angular.z = 0

    def BB(self):
        error = self.desired - self.actual
        # print(error)
        if error > 0:
            correction = self.k
        elif error < 0:
            correction = -self.k
        else:
            correction = 0
        self.twist.linear.x = self.v
        self.twist.angular.z = correction

    def P(self):
        error = float(self.desired - self.actual)
        print(error)
        correction = self.kp * error
        self.twist.linear.x = self.v
        self.twist.angular.z = correction
    
    def PI(self):
        error = float(self.desired - self.actual)
        self.integral += error
        print("error: " + str(error))
        print("last error: " + str(self.lasterror))
        if error == 0 or np.sign(error) != np.sign(self.lasterror):
            self.integral = 0
        self.integral = np.sign(self.integral) * min(abs(self.integral), self.maxIntegral)
        print(self.integral)
        print(abs(self.integral))
        print(self.maxIntegral)
        print(np.sign(self.integral))
        print('\n\n\n\n')

        correction = self.kp * error + self.ki * self.integral
        self.twist.linear.x = self.v
        self.twist.angular.z = correction
        self.lasterror = error

    def PID(self):
        error = float(self.desired - self.actual)
        self.integral += error
        print("error: " + str(error))
        print("last error: " + str(self.lasterror))
        if error == 0 or np.sign(error) != np.sign(self.lasterror):
            self.integral = 0
        self.integral = np.sign(self.integral) * min(abs(self.integral), self.maxIntegral)
        derivative = error - self.lasterror
        print("integral: ", self.integral)
        print("derivative: ", derivative)
        print('\n\n\n\n')
        self.twist.linear.x = self.v
        correction = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.twist.angular.z = correction
        self.lasterror = error

    def race(self): 
        error = float(self.desired - self.actual)
        self.integral += error
        if error == 0 or np.sign(error) != np.sign(self.lasterror):
            self.integral = 0
        self.integral = np.sign(self.integral) * min(abs(self.integral), self.maxIntegral)
        derivative = error - self.lasterror
        correction = self.kp * error + self.ki * self.integral + self.kd * derivative
        # self.twist.linear.x = self.v + min(1, abs(1/((correction * 3)**2 + 0.1)))
        # self.twist.linear.x = 1 / max(abs(float(error)),1) + self.v
        # self.twist.linear.x = self.v
        if self.state == STRAIGHT and derivative > 90:
            self.state = TURNING
        elif self.state == STRAIGHT:
            self.state = STRAIGHT
        elif self.state == TURNING and abs(error) < 20:
            self.state = STRAIGHT
        elif self.state == TURNING:
            self.state == TURNING
        
        if self.state == STRAIGHT:
            self.twist.linear.x = self.v + 0.4
        else:
            self.twist.linear.x = self.v
        self.twist.angular.z = correction
        self.lasterror = error
        
        self.log.write("error: " + str(error) + "\n")
        self.log.write("last error: " + str(self.lasterror) + "\n")
        self.log.write("integral: " + str(self.integral) + "\n")
        self.log.write("derivative: " + str(derivative) + "\n")
        self.log.write("angular speed: " + str(self.twist.angular.z) + "\n")
        self.log.write("speed: " + str(self.twist.linear.x) + "\n")
        self.log.write('================================================================\n')

    def close_log(self):
        self.log.close()


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab3')
    PID = PIDcontrol()
    try:
        while not rospy.is_shutdown():
            key = getKey()
            PID.follow_the_line()
            if (key == '\x03'): #stop the robot when exit the program
                break
    except rospy.ROSInterruptException:
        print("comm failed")
