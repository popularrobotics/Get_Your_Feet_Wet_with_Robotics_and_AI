#!/usr/bin/env python
# Popular Robotics Remote Control Code for course_w0
# Last edited by Lyuzhou Zhuang on 12/3/2018

"""
# This file enables the robot to be remotely controlled over Wifi from other devices.
#
# This file could also run on its own.
#
# PREREQUISITES
#	Tornado Web Server for Python
#
# Copyright
#   All rights reserved. No part of this script may be reproduced, distributed, or transmitted in any 
#   form or by any means, including photocopying, recording, or other electronic or mechanical methods, 
#   without prior written permission from Popular Robotics, except in the case of brief quotations embodied 
#   in critical reviews and certain other noncommercial uses permitted by copyright law. 
#   For permission requests, contact Popular Robotics directly.
#
# TROUBLESHOOTING:
#	  Don't use Ctrl+Z to stop the program, use Ctrl+c.
#	  If you use Ctrl+Z, it will not close the socket and you won't be able to run the program the next time.
#	  If you get the following error:
#		  "socket.error: [Errno 98] Address already in use "
#	  Run this on the terminal:
#		  "sudo netstat -ap |grep :9093"
#	  Note down the PID of the process running it
#	  And kill that process using:
#		  "kill pid"
#	  If it does not work use:
#		  "kill -9 pid"
#	  If the error does not go away, try changin the port number '9093' both in the client and server code
#
"""

import Motor_Remote_Control_Code as Remote

# Import the ArmRobot.py file (must be in the same directory as this file!).
from Adafruit_PWM_Servo_Driver import PWM
import RPi.GPIO as GPIO
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

application = tornado.web.Application([
    (r'/ws', Remote.WSHandler),
    (r'/', Remote.MainHandler),
    (r"/(.*)", tornado.web.StaticFileHandler, {"path": "./resources"}),
])


if __name__ == "__main__":
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)  # Numbers GPIOs by physical location
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
  
    pwm.setPWMFreq(50)  # Set frequency to 60 Hz
    running = True
    thread1 = Remote.myThread(1, "Thread-1", 1)
    thread1.setDaemon(True)
    thread1.start()
    application.listen(9093)    # starts the websockets connection
    tornado.ioloop.IOLoop.instance().start()
  

