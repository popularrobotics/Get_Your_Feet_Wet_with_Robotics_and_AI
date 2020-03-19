#!/usr/bin/env python
# Popular Robotics Online Competition Simulation Code
# Last edited by Sot Stas on Jan.16,2020

"""
# This file contains all the necessary code to receive the motion control commands by the Python program
# "course_w1.py" and transfer them to visualization program, but also to receive the sensor input data by the visualization program
# and return it to the program when requested.

#
# Copyright
#   All rights reserved. No part of this script may be reproduced, distributed, or transmitted in any 
#   form or by any means, including photocopying, recording, or other electronic or mechanical methods, 
#   without prior written permission from Popular Robotics, except in the case of brief quotations embodied 
#   in critical reviews and certain other noncommercial uses permitted by copyright law. 
#   For permission requests, contact Popular Robotics directly.
#
"""


import sys
import cv2
import rospy
import numpy
import rospkg
import os
from robot_control.msg import RobotMotion
from robot_control.msg import SensorInput
import time, json

robot_motion_msg = RobotMotion()
sensor_input_msg = SensorInput(500.0,500.0,500.0)

def callback(data):
    global sensor_input_msg
    sensor_input_msg = data

def initialize_simulation(left_sensor_angle=-15, front_sensor_angle=0, right_sensor_angle=15):
    rospy.init_node('competition_simulation', anonymous=True)

    global pub
    pub = rospy.Publisher("simulation/robot_motion", RobotMotion, queue_size=10)
    rospy.Subscriber("simulation/sensor_input", SensorInput, callback)

    pub_sensors = rospy.Publisher("simulation/sensor_orientation", SensorInput, queue_size=10)
    time.sleep(2) # delay some time to initialize the vizualization
    sensor_orientation_msg = SensorInput(left_sensor_angle,front_sensor_angle,right_sensor_angle)
    pub_sensors.publish(sensor_orientation_msg)
    time.sleep(3) # delay some time to initialize the vizualization of sensor orientation

# ----- wheel motor functions below -----

# motion_command 0-> stop_moving_for, 1-> go forward, 2-> go backward, 3-> spin left, 4-> spin right

def stop_moving_for(duration=0.0):

    # publish movement and speed
    robot_motion_msg.motion_command = 0
    robot_motion_msg.robot_speed = 0
    pub.publish(robot_motion_msg)

    time.sleep(duration)


def go_forward(speed=50, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)

    # publish movement and speed
    robot_motion_msg.motion_command = 1
    robot_motion_msg.robot_speed = speed
    pub.publish(robot_motion_msg)

    time.sleep(duration)


def go_backward(speed=30, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)

    # publish movement and speed
    robot_motion_msg.motion_command = 2
    robot_motion_msg.robot_speed = speed
    pub.publish(robot_motion_msg)

    time.sleep(duration)


def spin_left(speed=20, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)

    # publish movement and speed
    robot_motion_msg.motion_command = 3
    robot_motion_msg.robot_speed = speed
    pub.publish(robot_motion_msg)

    time.sleep(duration)


def spin_right(speed=20, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)

    # publish movement and speed
    robot_motion_msg.motion_command = 4
    robot_motion_msg.robot_speed = speed
    pub.publish(robot_motion_msg)

    time.sleep(duration)


# ----- wheel motor functions above -----

# ----- ultrasonic sensors input functions below -----

def read_all_distances():
    front_d = sensor_input_msg.front_sensor_distance
    left_d = sensor_input_msg.left_sensor_distance
    right_d = sensor_input_msg.right_sensor_distance
    return front_d, left_d, right_d


def initialize_ultrasonic_sensors():
    print '\nInitializing ultrasonic sensors.\n'
    front_distance, left_distance, right_distance = read_all_distances()
    print 'Front', front_distance, 'cm'
    print 'Right', right_distance, 'cm'
    print 'Left', left_distance, 'cm'
    print '\nUltrasonic sensors initialization completed.\n'

# ----- ultrasonic sensors input functions above -----

# -------------------------

if __name__ == "__main__":
     go_forward(20,1)
