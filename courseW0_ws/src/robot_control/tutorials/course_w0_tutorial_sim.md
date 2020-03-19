# Wheeled Robot DEMO Program Tutorial for Use with Simulation

## Overview
This tutorial explains all functions used in the python script you are going to edit. Please read through this tutorial as it contains useful information and could help you avoid a lot of potential bugs.
Here you are going to try out a function that controls the motors and makes the robot move.
You should NOT edit any existing functions other than student_function(), but you can always add new functions to the script.

---

## Instructions
Follow this guide to try out the motor function.
### STEP 1
Replace keyword `pass` in `student_function()` with `sim.go_forward(50, 0.5)` so that the robot moves forward at the speed of 50 for 0.5 seconds.

---

## Code Explained

The code below indicates that it's a Python program.
```python
#!/usr/bin/env python
```

The `import` command imports other python files (modules) to be used in this script
```python
import simulation as sim
import time
import sys
import cv2
import rospy
import numpy
import rospkg
import os
```

After you completed the previous steps and added in the code, the `student_function()` function now should look like this:
```python
def student_function():
    sim.initialize_simulation()
    sim.go_forward(50, 0.5)
    sim.stop_moving_for(0)

```
- `sim.initialize_simulation()` initializes the simulation environment.
- `sim.go_forward(50, 0.5)` means calling the `go_forward()` function from the `sim` simulation module that was imported at the beginning of the script.
The function is used with the following arguments:
`go_forward(speed, duration)`, where  `speed` is between 0 and 100 and `duration` is in seconds.
- The `stop_moving_for(duration)` function, where `duration` is in seconds, means stopping the vehicle from moving for `duration` seconds. We need this function to make our robot stop moving, otherwise it would keep executing the last command that we entered (e.g. go_forward).
---

The code below calls the `student_function()` once the program starts running:
```python
if __name__ == "__main__":
    if not rospy.is_shutdown():
        student_function()
```
Now unleash your creativity, write some code to drive the robot and make it go past the finish line!

---

## Copyright
All rights reserved. No part of this publication may be reproduced, distributed, or transmitted in any form or by any means, including photocopying, recording, or other electronic or mechanical methods, without prior written permission from Popular Robotics, except in the case of brief quotations embodied in critical reviews and certain other noncommercial uses permitted by copyright law. For permission requests, contact Popular Robotics directly.

---

Last edited by Sot on Jan.17,2020
