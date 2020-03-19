#!/usr/bin/env python
# Popular Robotics Wheeled Robot DEMO Program Template for Use with Simulation
# Last edited by Sotirios Stasinopoulos on Jan.17,2020

"""
# Copyright
#   All rights reserved. No part of this script may be reproduced, distributed, or transmitted in any
#   form or by any means, including photocopying, recording, or other electronic or mechanical methods,
#   without prior written permission from Popular Robotics, except in the case of brief quotations embodied
#   in critical reviews and certain other noncommercial uses permitted by copyright law.
#   For permission requests, contact Popular Robotics directly.
"""

# The below commands imports other python files (modules) to be used in this script.
import simulation as sim
import time
import sys
import cv2
import rospy
import numpy
import rospkg
import os

# ***** student edits start *****
# students define their own functions here.

def student_function():
    sim.initialize_simulation()
    pass
    sim.stop_moving_for(0)

# ***** student edits end *****


# ----- main function below -----

if __name__ == "__main__":
    student_function()

# ----- main function above -----
