#!/usr/bin/env python
# Popular Robotics CourseW1 Template
# Last edited by Lyuzhou Zhuang on 11/8/2018

"""
# This file contains all necessary code to read data from the ultrasonic
# sensors. It could run on its own or be imported as a module.
#
# Copyright
#   All rights reserved. No part of this script may be reproduced, distributed, or transmitted in any 
#   form or by any means, including photocopying, recording, or other electronic or mechanical methods, 
#   without prior written permission from Popular Robotics, except in the case of brief quotations embodied 
#   in critical reviews and certain other noncommercial uses permitted by copyright law. 
#   For permission requests, contact Popular Robotics directly.
#
"""

import RPi.GPIO as GPIO
import time

TRIG_F = 20     # BCM PIN settings
ECHO_F = 21
TRIG_R = 26
ECHO_R = 13
TRIG_L = 16
ECHO_L = 19


def setup_ultrasonic_GPIO():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_F, GPIO.OUT)
    GPIO.setup(ECHO_F, GPIO.IN)
    GPIO.setup(TRIG_R, GPIO.OUT)
    GPIO.setup(ECHO_R, GPIO.IN)
    GPIO.setup(TRIG_L, GPIO.OUT)
    GPIO.setup(ECHO_L, GPIO.IN)


def read_distance(side=0):
    time.sleep(0.05)
    if side == 1:   # read from right ultrasonic sensor
        trig = TRIG_R
        echo = ECHO_R
    elif side == 2: # read from left ultrasonic sensor
        trig = TRIG_L
        echo = ECHO_L
    else:           # read from front ultrasonic sensor
        trig = TRIG_F
        echo = ECHO_F

    GPIO.output(trig, 0)
    time.sleep(0.000002)

    GPIO.output(trig, 1)
    time.sleep(0.00001)
    GPIO.output(trig, 0)

    while_loop_starts = time.time()
    while GPIO.input(echo) == 0:
        if time.time() - while_loop_starts > 0.1:  # 0.5s
            # if entered this if statement, the ultrasonic sensor is probably malfunctioning
            print "\nultrasonic sensor", side, "seems to be malfunctioning,"
            return 2
    ultrasonic_signal_sent = time.time()

    while GPIO.input(echo) == 1:
        if time.time() - ultrasonic_signal_sent > 0.03:  # 0.03s
            # No need to worry about obstacles further than 5m away
            print "distance greater than 5m"
            return 500
    ultrasonic_signal_received = time.time()

    duration = ultrasonic_signal_received - ultrasonic_signal_sent
    return duration * 340 / 2 * 100  # cm


def read_all_distances():
    time.sleep(0.05)
    front_d = read_distance()
    left_d = read_distance(2)
    right_d = read_distance(1)
    return front_d, left_d, right_d


def initialize_ultrasonic_sensors():
    print '\nInitializing ultrasonic sensors.\n'
    front_distance, left_distance, right_distance = read_all_distances()
    print 'Front', front_distance, 'cm'
    print 'Right', right_distance, 'cm'
    print 'Left', left_distance, 'cm'
    print '\nUltrasonic sensors initialization completed.\n'


def loop_read_distance():
    while True:
        dis_f = read_distance(0)
        dis_r = read_distance(1)
        dis_l = read_distance(2)
        print 'Front', dis_f, 'cm'
        print 'Right', dis_r, 'cm'
        print 'Left', dis_l, 'cm'
        print ''
        time.sleep(0.3)


def destroy():
    GPIO.cleanup()


setup_ultrasonic_GPIO()

if __name__ == "__main__":
    setup_ultrasonic_GPIO()
    try:
        loop_read_distance()
    except KeyboardInterrupt:
        destroy()
