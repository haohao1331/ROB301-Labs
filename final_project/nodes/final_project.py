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
from datetime import datetime
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
(b, g, y, r) = (2, 1, 3, 0)
colors = {2: "blue", 1: 'green', 3: 'yellow', 0: 'red'}
state_color_map = np.array([y, r, g, b, b, r, g, b, y, r, g])

colour_codes = np.array([
    np.array([186.8, 115.2, 133.4]),  # red
    np.array([120.4, 165.6, 129.6]),  # green
    np.array([115.25, 133.25, 173.25]),  # blue
    np.array([203.8, 191.8, 126.6]),  # yellow
])

print(colour_codes.shape)

def normalize(a):
    return a / np.sum(a)

def get_max_index(a):
    return a.index(max(a))

stop_locations = np.arrary([2, 4, 7])
stop_locations -= 2

class BayesLoc:
    def __init__(self):
        self.colour_sub = rospy.Subscriber(
            "mean_img_rgb", UInt32MultiArray, self.colour_callback
        )
        self.line_sub = rospy.Subscriber("line_idx", String, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = len(state_color_map)
        self.probability = normalize(np.ones(self.num_states))
        self.state_prediction = np.zeros(self.num_states)

        self.cur_colour = None  # most recent measured colour

        # PID variables
        self.twist = Twist()
        self.actual = 320
        self.desired = 320
        self.v = 0.07
        self.k = 0.2
        self.kp = 0.004  # 0.025 -> perfect oscillation
        self.ki = 0.00001
        self.kd = 0.001
        self.integral = 0
        self.derivative = 0
        self.maxIntegral = 1000
        self.lasterror = 0
        self.on_color = False
        self.on_color_time = 43
        self.on_color_count = self.on_color_time

        # color testing
        self.colors = []
        self.now = datetime.now()
        current_time = self.now.strftime("%H_%M_%S")
        self.log = open('test_camera/' + current_time + '.txt', 'w+')
        
        # probability plot
        self.prob_data_file = open('prob_data/' + current_time + '.txt', 'w+')
        self.prob_data = []

        # stop
        self.stop_time = 20
        self.stop_counter = self.stop_time
        self.will_stop = False
        self.stopped = False


    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        self.cur_colour = np.array(msg.data)  # [r, g, b]
        # print("cur color: ", self.cur_colour)
        # self.colors.append(self.cur_colour)
    
    def test_stop(self):
        if max(self.probability) > 0.5 and get_max_index(self.probability) in stop_locations: 
            self.will_stop = True

    def PID(self):
        error = float(self.desired - self.actual)
        self.integral += error
        
        if error == 0 or np.sign(error) != np.sign(self.lasterror):
            self.integral = 0
        self.integral = np.sign(self.integral) * min(abs(self.integral), self.maxIntegral)
        self.derivative = error - self.lasterror
        correction = self.kp * error + self.ki * self.integral + self.kd * self.derivative
        line_color = np.array([160, 160, 160])
        if self.cur_colour != None and not self.on_color:
            diff = dot(self.cur_colour, line_color) / (norm(self.cur_colour) * norm(line_color))
            # print(diff)
            if diff < 0.99:
                self.on_color = True
                # print(self.cur_colour)
                dot_products = [False] * len(colour_codes)
                for i in range(len(colour_codes)):
                    dot_products[i] = dot(self.cur_colour, colour_codes[i]) / (norm(colour_codes[i]) * norm(self.cur_colour))
                # print(dot_products)
                index = get_max_index(dot_products)
                print("measured color: ", colors[index])
                self.colors.append(str(self.cur_colour) + "  " + str(diff) + "   " + str(dot_products) + " " + colors[dot_products.index(max(dot_products))])
                self.state_predict()
                self.state_update(index)
                self.test_stop()
        
        if self.on_color and not self.stopped:
            print("on color", self.on_color_count)
            correction = 0
            self.on_color_count += -1
            if self.on_color_count == self.on_color_time // 2 and self.will_stop:
                self.stopped = True
            if self.on_color_count == 0:
                self.on_color_count = self.on_color_time
                self.on_color = False
        
        if False:
            print("error: " + str(error))
            print("last error: " + str(self.lasterror))
            print("integral: ", self.integral)
            print("derivative: ", self.derivative)
            print("correction: ", correction)
            print('\n\n\n\n\n')
        # print(self.on_color)
        # print(self.on_color_count)
        # print("\n")

        self.twist.linear.x = self.v

        if self.stopped:
            print("stopped ", self.stop_counter)
            self.stop_counter -= 1
            correction = 0
            self.twist.linear.x = 0
            if self.stop_counter == 0:
                self.stopped = False
                self.stop_counter = self.stop_time
                self.will_stop = False

        self.twist.angular.z = correction
        self.lasterror = error


    def follow_the_line(self):
        self.PID()
        self.cmd_pub.publish(self.twist)

    def line_callback(self, msg):
        self.actual = int(msg.data)
        # print("line: ", self.actual)
        return

    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_predict(self):
        rospy.loginfo("predicting state")
        new = np.copy(self.probability)
        for i in range(len(new)):
            self.probability[i] = new[(i-1) % self.num_states]
        print("apriori update: ", self.probability)

    def state_update(self, measurement):
        rospy.loginfo("updating state")
        new_prob = np.zeros(self.num_states)
        for j in range(len(self.probability)):
            if state_color_map[j] == measurement:
                prob = 0.85
            else:
                prob = 0.05
            new_prob[j] = self.probability[j] * prob
        
        self.probability = normalize(new_prob)
        print("posterior update: ", self.probability)
        self.prob_data.append(np.copy(self.probability))

    def done(self):
        for s in self.colors:
            self.log.write(str(s) + "\n")
            # pass
        for s in self.prob_data:
            self.prob_data_file.write(str(s) + '\n')
        self.log.close()
        self.prob_data_file.close()


if __name__ == "__main__":

    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)

    localizer = BayesLoc()

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
