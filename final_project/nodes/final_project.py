#!/usr/bin/env python
from numpy.core.numeric import correlate
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt32MultiArray
import numpy as np
import colorsys
from numpy import dot
from numpy.linalg import norm


class BayesLoc:
    def __init__(self, p0, colour_codes, colour_map):
        self.colour_sub = rospy.Subscriber(
            "mean_img_rgb", UInt32MultiArray, self.colour_callback
        )
        self.line_sub = rospy.Subscriber("line_idx", String, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = len(p0)
        self.colour_codes = colour_codes
        self.colour_map = colour_map
        self.probability = p0
        self.state_prediction = np.zeros(self.num_states)

        self.cur_colour = None  # most recent measured colour

        # PID variables
        self.twist = Twist()
        self.actual = 320
        self.desired = 320
        self.v = 0.1
        self.k = 0.2
        self.kp = 0.004  # 0.025 -> perfect oscillation
        self.ki = 0.00000
        self.kd = 0.001
        self.integral = 0
        self.derivative = 0
        self.maxIntegral = 1000
        self.lasterror = 0
        self.on_color = False
        self.on_color_time = 32
        self.on_color_count = self.on_color_time

        # color testing
        self.colors = []
        self.log = open('log.txt', 'w')



    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        self.cur_colour = np.array(msg.data)  # [r, g, b]
        # print("cur color: ", self.cur_colour)
        self.colors.append(self.cur_colour)

    def PID(self):
        error = float(self.desired - self.actual)
        self.integral += error
        
        if error == 0 or np.sign(error) != np.sign(self.lasterror):
            self.integral = 0
        self.integral = np.sign(self.integral) * min(abs(self.integral), self.maxIntegral)
        self.derivative = error - self.lasterror
        if False:
            print("error: " + str(error))
            print("last error: " + str(self.lasterror))
            print("integral: ", self.integral)
            print("derivative: ", self.derivative)
            print('\n\n\n\n\n\n')
        self.twist.linear.x = self.v
        correction = self.kp * error + self.ki * self.integral + self.kd * self.derivative
        # correction = 0
        line_color = np.array([160, 160, 160])
        if self.cur_colour != None and not self.on_color:
            diff = dot(self.cur_colour, line_color) / (norm(self.cur_colour) * norm(line_color))
            print(diff)
            if diff < 0.99:
                self.on_color = True
                
        
        if self.on_color:
            correction = 0
            self.on_color_count += -1
            if self.on_color_count == 0:
                self.on_color_count = self.on_color_time
                self.on_color = False
        
        print(self.on_color)
        print(self.on_color_count)
        print("\n")

        self.twist.angular.z = correction
        self.lasterror = error



    def follow_the_line(self):
        self.PID()
        # self.test()
        self.cmd_pub.publish(self.twist)
        # self.rate.sleep()

    def line_callback(self, msg):
        self.actual = int(msg.data)
        # print("line: ", self.actual)
        return

    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_model(self, u):
        """
        State model: p(x_{k+1} | x_k, u)

        TODO: complete this function
        """

    def measurement_model(self, x):
        """
        Measurement model p(z_k | x_k = colour) - given the pixel intensity,
        what's the probability that of each possible colour z_k being observed?
        """
        if self.cur_colour is None:
            self.wait_for_colour()

        # prob = np.zeros(len(colourCodes))

        """
        TODO: You need to compute the probability of states. You should return a 1x5 np.array
        Hint: find the euclidean distance between the measured RGB values (self.cur_colour)
            and the reference RGB values of each colour (self.ColourCodes).
        """

        # return prob

    def state_predict(self):
        rospy.loginfo("predicting state")
        """
        TODO: Complete the state prediction function: update
        self.state_prediction with the predicted probability of being at each
        state (office)
        """

    def state_update(self):
        rospy.loginfo("updating state")
        """
        TODO: Complete the state update function: update self.probabilities
        with the probability of being at each state
        """

    def done(self):
        for s in self.colors:
            # self.log.write(str(s) + "\n")
            pass
        self.log.close()


if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: red, 1: green, 2: blue, 3: yellow, 4: line
    # current map starting at cell #2 and ending at cell #12
    colour_map = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1]

    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    colour_codes = [
        [167, 146, 158],  # red
        [163, 184, 100],  # green
        [173, 166, 171],  # blue
        [167, 170, 117],  # yellow
        [150, 150, 150],  # line
    ]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)

    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.init_node("final_project")
    rospy.sleep(0.5)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        """
        TODO: complete this main loop by calling functions from BayesLoc, and
        adding your own high level and low level planning + control logic
        """
        localizer.follow_the_line()
        rate.sleep()

    localizer.done()
    rospy.loginfo("finished!")
    rospy.loginfo(localizer.probability)
