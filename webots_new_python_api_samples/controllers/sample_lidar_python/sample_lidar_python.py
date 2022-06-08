"""This is a "new Python API" sample using a Webots lidar device.
   This requires numpy, installable with 'pip install numpy'.
   This sample illustrates transferring lidar readings to numpy for fast numerical processing, converting readings
   to images to show on a Webots pop-up display, and doing crude wayfinding using the range data to avoid obstacles
   and seek distant goals.  Tapping C toggles between displaying the simple range readings (grayscale) and
   displaying colorful pointcloud readings.  In the colorful pointcloud, obstacles appear the default grey since their
   x/y/z distances are close to the default zero.  The background near the left and right edges is quite blue, since
   these represent points far ahead of the robot in the +x/+blue/+forward direction.  The reddest points are in the
   background 3/4 of the way across, corresponding to the robot's +y/+red/+left direction.  And the greenest/yellowest
   points are high on distant walls as those are highest-up points lidar beams hit in the +z/+green/+up direction."""

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

TIME_STEP = 64    # How many milliseconds between each time the controller issues commands
SPEED = 3         # Maximum speed robot will move
EMERGENCY = 0.15  # If the robot ever gets this close to anything, it will swivel hard to get away
CAUTION = 0.5     # The robot will try to steer around anything that gets this close
DEADEND = 2       # If all points ahead of the robot are this close, it will turn away


# === Initialize Devices ===

keyboard = robot.keyboard  # give keyboard local name for easy reference; this also automatically enables it

motors = Pair(robot.Motor)     # Now motors.left will be the robot's first motor, motors.right the second
motors.target_position = None  # set both motors not to seek any target; we'll control them by velocity instead
motors.velocity = 0.0          # set both motors to be stopped, for now

lidar = robot.Lidar()
lidar.cloud.sampling = True  # now we can access point-cloud readings as well as basic range images

display = robot.Display()
stretch = display.height // lidar.height  # how many vertical pixels each lidar pixel will occupy to fill display

print_readings = False
print("Tap P to toggle printing of the closest distances on left and right, which largely drive the robot's decisions.")
display_cloud = False
print("Tap C to toggle whether display shows colorful point-cloud readings or drab range readings.")

# === Main loop ===
while robot.step(TIME_STEP):
    # --- Keyboard keys 'P' and 'C' toggle options ---
    if 'P' in robot.keyboard.pressed:  # tapping P toggles printing current min readings
        print_readings = not print_readings
    if 'C' in robot.keyboard.pressed:  # tapping C toggles whether display shows colorful cloud or drab range
        display_cloud = not display_cloud

    # --- Display Lidar readings on a Webots display ---
    if display_cloud:  # colorful cloud image
        img_BGRA = lidar.cloud.color_image(x_min = -5, x_max = 6, y_min = -5, y_max = 6, z_min=-1, z_max=4,
                                           x_color = (255,0,0), y_color=(0,0,255), z_color=(0,255,0))
    else:
        img_BGRA = lidar.color_image(min_color=(0,0,0), max_color=(255,255,50))
    # This lidar has 6 layers, but a 6-pixel-tall image would be hard to view, so we'll stretch it vertically
    display.paste(img_BGRA.repeat(stretch, axis=0))

    # --- read the Lidar's range image into a numpy array ---

    img_f = lidar.array   # floats ranging 0..8; arranged in 6 rows of 256 ringing robot, from straight ahead->clockwise

    # --- use Lidar range readings for wayfinding---
    # Flatten 6 layers of range readings into one, for simplicity; we take the min since Lidar sees over some blocks
    distances = np.min(img_f, axis=0)  # line of 256 floats ranging 0..8; starting straight ahead, scanning to right
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
        left, right = img_f[3,-40:], img_f[3,:40]  # look out level (lidar layer 3) for most distant point
        if max(right) > max(left):
            motors.velocity = SPEED, SPEED/2
        else:
            motors.velocity = SPEED/2, SPEED

    if print_readings: print(min_left, min_right)