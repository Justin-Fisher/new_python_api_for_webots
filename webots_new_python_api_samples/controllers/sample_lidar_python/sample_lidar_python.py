"""This is a "new Python API" sample using a Webots lidar device.
   This requires numpy, installable with 'pip install numpy'.
   This sample illustrates transferring lidar readings to numpy for fast numerical processing, converting readings
   to images to show on a Webots pop-up display, and doing crude wayfinding using the range data to avoid obstacles
   and seek distant goals."""

 # * Copyright 2022 Justin Fisher.
 # *
 # * Licensed under the Apache License, Version 2.0 (the "License");
 # * you may not use this file except in compliance with the License.
 # * You may obtain a copy of the License at
 # *
 # *     http://www.apache.org/licenses/LICENSE-2.0
 # *
 # * Unless required by applicable law or agreed to in writing, software
 # * distributed under the License is distributed on an "AS IS" BASIS,
 # * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # * See the License for the specific language governing permissions and
 # * limitations under the License.

import numpy as np
import robot
from vectors import Pair  # convenient container for managing a left-right pair of similar devices, like Motors

SPEED = 3                       # Maximum speed robot will move
TIME_STEP = 64                  # How many milliseconds between each time the controller issues commands

# === Initialize Devices ===

display = robot.Display()  # When there is only one instance of a device class, providing its name is optional

keyboard = robot.keyboard  # give keyboard local name for easy reference; this also automatically enables it

motors = Pair(robot.Motor)     # Now motors.left will be the robot's first motor, motors.right the second
motors.target_position = None  # set both motors not to seek any target; we'll control them by velocity instead
motors.velocity = 0.0          # set both motors to be stopped, for now

lidar = robot.Lidar()
lidar.enablePointCloud()

display = robot.Display()

# Create array in which we'll build the image to show on the display
img_BGRA = np.zeros((lidar.height, lidar.width, 4), dtype='B')
img_BGRA[:,:,3]=255  # set alpha layer to be opaque

print_readings = False
print("Tap P to toggle printing of the closest distances on left and right, which largely drive the robot's decisions.")

EMERGENCY = 0.8  # If we ever get this close to anything, we swivel hard to get away
CAUTION = 2.5    # Anything that gets this close we'll try to steer around
DEADEND = 10     # If all points ahead of us are this close, turn away

# === Main loop ===
while robot.step(TIME_STEP):
    # --- present Lidar readings on a Webots display ---
    img_f = lidar.array                          # floats ranging 0..8; arranged in 6 rows of 256 ringing around robot
    img_b = np.array(img_f*(255/8), dtype='B')   # bytes ranging 0..255
    img_b.resize((lidar.height, lidar.width, 1)) # view as having thin 3rd dimension to allow broadcasting
    img_BGRA[:,:,0:2] = img_b                    # copy range info onto BG channels
    display.Image(img_BGRA).paste_once()         # paste this onto the display

    # --- use Lidar range readings for wayfinding---
    # Flatten the 6 layers of readings into one, for simplicity; we take the min since Lidar sees over some blocks
    distances = np.min(img_f, axis=0)*6  # line of 256 floats ranging 0..48; starting straight ahead, scanning to right
    # Check if we're about to run into anything; if so, spin hard to the right to find a clearer path
    left, right = distances[-40:], distances[:40]  # each contains distances within ~50 degrees of ahead on that side
    min_left, min_right = min(left), min(right)    # to avoid repeatedly computing
    if min_left < EMERGENCY or min_left == float('inf') or min_right < EMERGENCY or min_right==float('inf'):
        motors.velocity = SPEED, -SPEED
    # If we can see that we're headed into a dead end, also should turn away, but more gently is fine
    elif all(left < DEADEND) and all(right < DEADEND):
        motors.velocity = SPEED, -SPEED/4
    # If we can see obstacles on both sides, but think there's a path, then try to slowly pass between
    elif min_left < CAUTION and min_right < CAUTION:
        if min_left < min_right:
            motors.velocity = SPEED*0.7, SPEED*0.2
        else:
            motors.velocity = SPEED*0.2, SPEED*0.7
    # If we can see obstacles approaching on just one side, try to steer clear of them
    elif min_left < CAUTION:
        motors.velocity = SPEED, SPEED/4
    elif min_right < CAUTION:
        motors.velocity = SPEED/4, SPEED
    # Otherwise if we're in the clear, we try to head towards the furthest point we can see roughly ahead of us
    else:
        if max(right) > max(left):
            motors.velocity = SPEED, SPEED/2
        else:
            motors.velocity = SPEED/2, SPEED

    if 'P' in robot.keyboard.pressed:  # tapping P toggles printing current min readings
        print_readings = not print_readings

    if print_readings: print(min_left, min_right)