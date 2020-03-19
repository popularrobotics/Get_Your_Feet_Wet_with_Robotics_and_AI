#!/usr/bin/env python
# Popular Robotics Course Template
# Last edited by Lyuzhou Zhuang on 4/12/2019

"""
# This file contains all the necessary code to drive the motor part of
# the robot for course_W1.
#
# This file could also run on its own
#
# Copyright
#   All rights reserved. No part of this script may be reproduced, distributed, or transmitted in any 
#   form or by any means, including photocopying, recording, or other electronic or mechanical methods, 
#   without prior written permission from Popular Robotics, except in the case of brief quotations embodied 
#   in critical reviews and certain other noncommercial uses permitted by copyright law. 
#   For permission requests, contact Popular Robotics directly.
#
"""


# import rospy
# from std_msgs.msg import String
import time, json

# Import the Adafruit_PWM_Servo_Driver.py file (must be in the same directory as this file!).
from Adafruit_PWM_Servo_Driver import PWM
import RPi.GPIO as GPIO  
import threading
import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.template
import tornado.escape

# Initialise the PWM device using the default address
pwm = PWM(0x40)
servoMin = 150  # Min pulse length out of 4096  #150
servoMax = 600  # Max pulse length out of 4096 #600

PWMA = 18
AIN1 = 22
AIN2 = 27

PWMB = 23
BIN1 = 25
BIN2 = 24

arm_positions = [35, 30, 170, 160]  # initial arm position
# 0 for Claw, 35-90 degrees
# 1 for Forearm, in or out, 22-160 degrees
# 2 for Upper Arm, up or down, 90-175 degrees
# 3 for Arm Orientation, left or right, 0-160 degrees


# ----- wheel motor functions below -----

def go_forward(speed=50, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, False)    # AIN2
    GPIO.output(AIN1, True)     # AIN1

    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, False)    # BIN2
    GPIO.output(BIN1, True)     # BIN1
    time.sleep(duration)


def stop_moving_for(duration=0.0):
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN2, False)    # AIN2
    GPIO.output(AIN1, False)    # AIN1

    R_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN2, False)    # BIN2
    GPIO.output(BIN1, False)    # BIN1
    time.sleep(duration)


def go_backward(speed=30, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, True)     # AIN2
    GPIO.output(AIN1, False)    # AIN1

    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, True)     # BIN2
    GPIO.output(BIN1, False)    # BIN1
    time.sleep(duration)


def spin_left(speed=20, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, True)     # AIN2
    GPIO.output(AIN1, False)    # AIN1

    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, False)    # BIN2
    GPIO.output(BIN1, True)     # BIN1
    time.sleep(duration)


def turn_left(speed=50, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)
    L_Motor.ChangeDutyCycle(speed/2)
    GPIO.output(AIN2, False)     # AIN2
    GPIO.output(AIN1, True)    # AIN1

    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, False)    # BIN2
    GPIO.output(BIN1, True)     # BIN1
    time.sleep(duration)


def backward_left(speed=50, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)
    L_Motor.ChangeDutyCycle(speed/2)
    GPIO.output(AIN2, True)     # AIN2
    GPIO.output(AIN1, False)    # AIN1

    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, True)    # BIN2
    GPIO.output(BIN1, False)     # BIN1
    time.sleep(duration)


def spin_right(speed=20, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, False)    # AIN2
    GPIO.output(AIN1, True)     # AIN1

    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, True)     # BIN2
    GPIO.output(BIN1, False)    # BIN1
    time.sleep(duration)


def turn_right(speed=50, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, False)     # AIN2
    GPIO.output(AIN1, True)    # AIN1

    R_Motor.ChangeDutyCycle(speed/2)
    GPIO.output(BIN2, False)    # BIN2
    GPIO.output(BIN1, True)     # BIN1
    time.sleep(duration)


def backward_right(speed=50, duration=0.0):
    speed = max(speed, 0)
    speed = min(speed, 100)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, True)     # AIN2
    GPIO.output(AIN1, False)    # AIN1

    R_Motor.ChangeDutyCycle(speed/2)
    GPIO.output(BIN2, True)    # BIN2
    GPIO.output(BIN1, False)     # BIN1
    time.sleep(duration)


# ----- wheel motor functions above -----
# ----- arm motor functions below -----


def set_servo_pulse(channel, pulse):
    pulse_length = 1000000.0    # 1, 000, 000 us per second
    pulse_length /= 50.0        # 60 Hz
    # print "%d us per period" % pulse_length
    pulse_length /= 4096.0      # 12 bits of resolution
    # print "%d us per bit" % pulse_length
    pulse *= 1000.0
    pulse /= (pulse_length*1.0)
    # pwmV=int(pulse)
    # print "pulse: %f  " % pulse
    pwm.setPWM(channel, 0, int(pulse))


# Calculate Forearm Angle limit
def cal_forearm_angle_limit(current_upper_arm_angle):
    x = current_upper_arm_angle / 10 - 8
    # print "Upper Arm:", current_upper_arm_angle
    max_angle = int(-0.9134 * x ** 2 + 3.2009 * x + 153.81 - 2)
    # print "Forearm Limit Max:", max_angle
    min_angle = int(0.0584 * x ** 2 - 3.4511 * x + 60.071 + 3)
    # print "Forearm Limit Min:", min_angle
    return min_angle, max_angle


# Arm Angle to PWM
def arm_angle_to_motor_pwm(arm_angle):
    arm_pwm = arm_angle / 90.0 + 0.5
    arm_pwm = max(arm_pwm, 0.5)
    arm_pwm = min(arm_pwm, 2.5)  # Constrain the value to be between 0.5 and 2.5
    return arm_pwm


# Move the arm smoothly
def move_the_arm_smoothly(servo_num, angle1, angle2):
    if angle1 <= angle2:
        for i in range(int(angle1), int(angle2) + 1):
            time.sleep(0.01)
            servo_pwm = arm_angle_to_motor_pwm(i)
            set_servo_pulse(servo_num, servo_pwm)
    else:
        for i in range(int(angle1), int(angle2) - 1, -1):
            time.sleep(0.01)
            servo_pwm = arm_angle_to_motor_pwm(i)
            set_servo_pulse(servo_num, servo_pwm)


# Move the arm swiftly
def move_the_arm_swiftly(servo_num, angle):
    time.sleep(0.1)
    servo_pwm = arm_angle_to_motor_pwm(angle)
    set_servo_pulse(servo_num, servo_pwm)


# Move Arm to Set Angle
def arm_position(servo_num, servo_angle):
    global arm_positions

    # Set angle limit for the forearm
    if servo_num == 1:  # If moving the forearm
        servo_angle = max(22, servo_angle)
        servo_angle = min(160, servo_angle)
        # Calculate forearm angle limit
        min_angle, max_angle = cal_forearm_angle_limit(arm_positions[2])
        if servo_angle < min_angle:
            servo_angle = min_angle
            if arm_positions[1] < min_angle:
                move_the_arm_swiftly(1, servo_angle)
                arm_positions[1] = int(servo_angle)
        elif servo_angle > max_angle:
            servo_angle = max_angle
            if arm_positions[1] > max_angle:
                move_the_arm_swiftly(1, servo_angle)
                arm_positions[1] = int(servo_angle)

    if servo_num == 2:  # If moving the upper arm
        servo_angle = max(90, servo_angle)
        servo_angle = min(175, servo_angle)
        # Calculate forearm angle limit
        min_angle, max_angle = cal_forearm_angle_limit(servo_angle)
        if arm_positions[1] < min_angle:
            move_the_arm_smoothly(1, arm_positions[1], min_angle)
            arm_positions[1] = int(min_angle)
        elif arm_positions[1] > max_angle:
            move_the_arm_smoothly(1, arm_positions[1], max_angle)
            arm_positions[1] = int(max_angle)

    if servo_num == 0:  # 0 for Claw, 35-90 degrees
        servo_angle = max(35, servo_angle)
        servo_angle = min(90, servo_angle)

    if servo_num == 3:  # 3 for Arm Orientation, left or right, 0-160 degrees
        servo_angle = max(0, servo_angle)
        servo_angle = min(160, servo_angle)

    # Move arm from previous position to new position
    move_the_arm_smoothly(servo_num, arm_positions[servo_num], servo_angle)
    # Save new position
    arm_positions[servo_num] = int(servo_angle)


# ----- Specific arm functions below -----
# Have functions dedicated to each arm motors
# so there's no need to remember motor numbers.

def claw(angle=80):
    arm_position(0, angle)


def forearm(angle=30):
    arm_position(1, angle)


def upper_arm(angle=170):
    arm_position(2, angle)


def arm_orientation(angle=160):
    arm_position(3, angle)

# ----- Specific arm functions above -----


def arm_returns_to_default_position():
    global arm_positions
    print "\nArm returning to its default position..."
    arm_position(0, 80)   # 0 for Claw, 35-90 degrees
    arm_position(1, 30)   # 1 for Forearm, in or out, 22-160 degrees
    arm_position(2, 170)  # 2 for Upper Arm, up or down, 90-175 degrees
    arm_position(3, 160)  # 3 for Arm Orientation, left or right, 0-160 degrees
    arm_position(1, 30)   # 1 for Forearm, in or out, 22-160 degrees
    arm_position(0, 35)   # 0 for Claw, 35-90 degrees
    arm_positions = [35, 30, 170, 160]  # initial arm position
    print "Arm returned to its default position.\n"


# ----- arm motor functions above -----


def initialize_arm_motors():
    # Put arm to default position before starting to use the arm
    arm_returns_to_default_position()


def destroy():
    arm_returns_to_default_position()
    GPIO.cleanup()


###############################


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)       # Numbers GPIOs by physical location
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)

GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)

L_Motor = GPIO.PWM(PWMA, 100)
L_Motor.start(0)
R_Motor = GPIO.PWM(PWMB, 100)
R_Motor.start(0)

pwm.setPWMFreq(50)                        # Set frequency to 60 Hz
running = True


def test_wheel_motors():    # test to see if wheel motors function properly
    go_forward(50, 0.5)  # arguments: speed, duration (seconds)
    stop_moving_for(.2)  # argument: duration (seconds)
    go_backward(50, 0.5)
    stop_moving_for(.2)
    spin_left(50, 0.5)
    stop_moving_for(.2)
    spin_right(50, 0.5)
    stop_moving_for(.2)


def test_arm_motors():  # test to see if arm motors function properly
    print "Arm test starts"

    print "Arm points forward"
    arm_position(3, 80)  # 3 for Arm Orientation, left or right, 0-160 degrees

    print "Claw opens"
    arm_position(0, 90)   # 0 for Claw, 35-90 degrees
    print "Claw closes"
    arm_position(0, 40)  # 0 for Claw, 35-90 degrees

    print "Upper Arm up"
    arm_position(2, 130)   # 2 for Upper Arm, up or down, 90-175 degrees
    print "Upper Arm down"
    arm_position(2, 170)  # 2 for Upper Arm, up or down, 90-175 degrees

    print "Forearm out"
    arm_position(1, 80)  # 1 for Forearm, in or out, 22-160 degrees, degree limits depend on Upper Arm position
    print "Forearm in"
    arm_position(1, 30)  # 1 for Forearm, in or out, 22-160 degrees, degree limits depend on Upper Arm position

    print "Arm points left"
    arm_position(3, 150)   # 3 for Arm Orientation, left or right, 0-160 degrees

    print "Arm test completes"
    time.sleep(1)

    arm_returns_to_default_position()


# -------------------------

if __name__ == "__main__":
    try:
        test_wheel_motors()
        #initialize_arm_motors()
        #test_arm_motors()
    except KeyboardInterrupt:
        destroy()
