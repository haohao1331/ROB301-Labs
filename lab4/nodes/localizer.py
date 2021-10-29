#!/usr/bin/env python
from math import cos
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import String


class PIDcontrol():

    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.color_sub = rospy.Subscriber('line_idx', String, self.camera_callback, queue_size=1)
        self.cmd_sub = rospy.Subscriber("cmd_vel_noisy", Twist, self.cmd_callback)
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
        self.state = 0
        self.adj_counter = 0
        self.adj_threshold = 5
        self.delta_x = 0.0
        self.u = 0.1
        self.prev_time = rospy.Time.now().to_sec()

    def camera_callback(self, data):
        self.actual = int(data.data)
        # print(self.actual)
    
    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x
        # print(self.u)

    def follow_the_line(self):
        self.PID()
        # self.test()
        self.cmd_pub.publish(self.twist)
        self.rate.sleep()

    def unfiltered(self):
        print((1.0/30.0)*float(self.u))
        now = rospy.Time.now().to_sec()
        self.delta_x = self.delta_x + (now - self.prev_time)*float(self.u)*100.0 # rate = 30 Hz
        print(self.delta_x)
        self.prev_time = now
        return float(self.delta_x)

    def test(self):
        self.twist.linear.x = self.v
        self.twist.angular.z = 0

    def PID(self):
        error = float(self.desired - self.actual)
        self.integral += error
        # print("error: " + str(error))
        # print("last error: " + str(self.lasterror))
        if error == 0 or np.sign(error) != np.sign(self.lasterror):
            self.integral = 0
        self.integral = np.sign(self.integral) * min(abs(self.integral), self.maxIntegral)
        derivative = error - self.lasterror
        # print("integral: ", self.integral)
        # print("derivative: ", derivative)
        # print('\n\n\n\n')
        self.twist.linear.x = self.v
        correction = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.twist.angular.z = correction
        self.lasterror = error
        

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = x_0
        self.x_prior = np.nan
        self.dt = 1 # TODO

        self.ycn = 1 # TODO
        self.xcn = 1 # TODO

        self.u = 0.1  # initialize the cmd_vel input
        self.phi = np.nan  # initialize the measurement input

        self.state_pub = rospy.Publisher("state", Float64, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            "scan_angle", Float64, self.scan_callback, queue_size=1
        )
        self.cmd_sub = rospy.Subscriber("cmd_vel_noisy", Twist, self.cmd_callback)

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, msg):
        self.phi = msg.data

    ## call within run_kf to update the state with the measurement
    def predict(self, u=0):
        D = self.ycn / ((self.xcn - self.x)**2 + self.ycn**2)
        self.x = self.x + u * self.dt
        self.P = self.P + self.Q
        S = D * self.P * D + self.R
        W = self.P * D / S
        self.P = self.P - W * D * self.P
        

        """
        TODO: update state via the motion model, and update the covariance with the process noise
        """
        return

    ## call within run_kf to update the state with the measurement
    def measurement_update(self):
        self.phi = np.arctan(self.ycn / (self.xcn - self.x))

        """
        TODO: update state when a new measurement has arrived using this function
        """
        return

    def run_kf(self):
        current_input = self.u
        current_measurement = self.phi

        """
        TODO: complete this function to update the state with current_input and current_measurement
        """

        self.state_pub.publish(self.x)




if __name__ == "__main__":
    rospy.init_node("lab4")
    PID = PIDcontrol()
    h = 0.60  # y distance to tower
    d = 0.60 * 3  # x distance to tower (from origin)

    x_0 = 0  # initial state position

    Q = 1  # TODO: Set process noise covariance
    R = 1  # TODO: measurement noise covariance
    P_0 = 1  # TODO: Set initial state covariance

    rate = 30 #Hz

    kf = KalmanFilter(h, d, x_0, Q, R, P_0)
    rospy.sleep(1)

    rate = rospy.Rate(rate)
    th = 0.5
    counter = 0
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    for i in [61, 122, 244, 305]:
        print(i)
        while not rospy.is_shutdown():
            PID.follow_the_line()

            if i - th <= PID.unfiltered() <= i + th:
                
                break
            #kf.run_kf()
            rate.sleep()
            

        
        while not rospy.is_shutdown():
            counter += 1
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            cmd_pub.publish(twist)

            print(i)
            
            if counter == 60:
                counter = 0
                break
            #kf.run_kf()
            rate.sleep()
    print("done")
