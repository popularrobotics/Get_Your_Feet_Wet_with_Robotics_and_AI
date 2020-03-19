# Wheeled Robot DEMO Program Template for Use with Real Robot

## Overview
This tutorial explains all functions used in the python script you are going to edit. Please read through this tutorial as it contains useful information and could help you avoid a lot of potential bugs.
Here you are going to try out a function that controls the motors and makes the robot move.
You should NOT edit any existing functions other than student_function(), but you can always add new functions to the script.

---

## Instructions
Follow this guide to try out the motor function.
### STEP 1
Replace keyword `pass` in `student_function()` with `robot.go_forward(50, 0.5)` so that the robot moves forward at the speed of 50 for 0.5 seconds.

---

## Code Explained

The code below indicates that it's a Python program.
```python
#!/usr/bin/env python
```

The `import` command imports other python files (modules) to be used in this script
```python
import Basic_Motor_Control_Code as robot
```

After you completed the previous steps and added in the code, the `student_function()` function now should look like this:
```python
def student_function():
    robot.go_forward(50, 0.5)

```
`robot.go_forward(50, 0.5)` means calling the `go_forward()` function from the `robot` module that was imported at the beginning of the script.
The function is used with the following arguments:
`go_forward(speed, duration)`, where  `speed` is between 0 and 100 and `duration` is in seconds.

---

The code below calls the `student_function()` once the program starts running:
```python
if __name__ == "__main__":
    if not rospy.is_shutdown():
        student_function()
```
Now unleash your creativity, write some code to drive the robot!

---

## Copyright
All rights reserved. No part of this publication may be reproduced, distributed, or transmitted in any form or by any means, including photocopying, recording, or other electronic or mechanical methods, without prior written permission from Popular Robotics, except in the case of brief quotations embodied in critical reviews and certain other noncommercial uses permitted by copyright law. For permission requests, contact Popular Robotics directly.

---

Edited by Lyuzhou Zhuang on 6/20/2019
