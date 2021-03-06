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
import world
from vectors import Pair  # convenient container for managing a left-right pair of similar devices, like Motors

TIME_STEP = 64    # How many milliseconds between each time the controller issues commands
SPEED = 3         # Maximum speed robot will move
EMERGENCY = 0.15  # If the robot ever gets this close to anything, it will swivel hard to get away
CAUTION = 0.5     # The robot will try to steer around anything that gets this close
DEADEND = 2       # If all points ahead of the robot are this close, it will turn away


# === Initialize Devices ===

display = robot.Display()  # When there is only one instance of a device class, providing its name is optional

keyboard = robot.keyboard  # give keyboard local name for easy reference; this also automatically enables it

motors = Pair(robot.Motor)     # Now motors.left will be the robot's first motor, motors.right the second
motors.target_position = None  # set both motors not to seek any target; we'll control them by velocity instead
motors.velocity = 0.0          # set both motors to be stopped, for now

lidar = robot.Lidar()
lidar.cloud  # referring to this automatically enables point-cloud readings


# Create arrays in which we'll build the images to show on the display
img_BGRA = np.zeros((lidar.height, lidar.width, 4), dtype='B')
img_BGRA[:,:,3]=255  # set alpha layer to be opaque
cloud_xzy = np.zeros((lidar.height, lidar.width, 3), dtype='<f4')  # colorful image looks better with x=blue, z=green, y=red
cloud_BGRA = img_BGRA.copy()
thickness = display.height // lidar.height  # how many vertical pixels each lidar pixel will occupy on display
thick_BGRA = np.zeros((thickness * lidar.height, lidar.width, 4), dtype='B')
# another view of thick_BGRA, now projecting its thickness into a 4th dimension; easier to broadcast "thin" image into
folded_BGRA = thick_BGRA.reshape(lidar.height, thickness, lidar.width, 4)
thin_BGRA = img_BGRA.reshape(lidar.height, 1, lidar.width, 4)  # another view of img_BGRA, broadcastable onto folded_BGRA

print(f"{cloud_xzy.shape=}")
print(f"{img_BGRA.shape=}")
print(f"{folded_BGRA.shape=}")
print(f"{thick_BGRA.shape=}")

print_readings = False
print("Tap P to toggle printing of the closest distances on left and right, which largely drive the robot's decisions.")
display_cloud = False
print("Tap C to toggle whether display shows colorful point-cloud readings or drab range readings.")

# === Plan a visual representation of the point cloud as a grid drawn in the display ===
LINE_COLOR = (0.5,0.5,0.5)
width, height = lidar.width, lidar.height
plan = world.plan
# We'll connect the grid across each row looping around, and down each column
row_indices = [list(range(y*width, (y+1)*width))+[y*width,  -1] for y in range(height)]
col_indices = [list(range(x, x + width*height, width)) + [-1] for x in range(width )]
indices = sum(col_indices, []) + sum(row_indices, [])  # flatten these into a single long list

lineset_plan = plan.IndexedLineSet(DEF = "CLOUD_LINESET",
                                   material = plan.Material(diffuseColor=LINE_COLOR, emissiveColor=LINE_COLOR),
                                   coord = plan.Coordinate(point = [(0, 0, 0)] * lidar.height * lidar.width),
                                   coordIndex = indices
                                  )
world.Node(lidar).children.append(lineset_plan)

# Store convenient references to relevant CLOUD_LINESET fields.
point_field = world.Node(lidar).CLOUD_LINESET.coord.point  # list of 3D coordinates of points to display

# === Main loop ===
while robot.step(TIME_STEP):
    # --- Keyboard keys 'P' and 'C' toggle options ---
    if 'P' in robot.keyboard.pressed:  # tapping P toggles printing current min readings
        print_readings = not print_readings
    if 'C' in robot.keyboard.pressed:  # tapping C toggles whether display shows colorful cloud or drab range
        display_cloud = not display_cloud

    # --- read the Lidar's range image into a numpy array ---

    img_f = lidar.array   # floats ranging 0..8; arranged in 6 rows of 256 ringing robot, from straight ahead->clockwise
    cloud = lidar.cloud.array  # shape (height x width x 3): floats indicating x,y,z coords of each seen point

    # --- present Lidar readings on a Webots display ---
    if display_cloud:
        cloud_xzy[...,0] = cloud[...,0]     # keep the cloud's x/forwards/0 cloud at the blue/0 position
        cloud_xzy[...,1] = cloud[...,2]*6-2 # shift the cloud's z/up/2 to the green/1 position, accentuated due to flat world
        cloud_xzy[...,2] = cloud[...,1]     # shift the cloud's y/left/1 to the red/2 position
        cloud_b = np.array((cloud_xzy+8)*(255/16), dtype='B')  # convert to bytes ranging 0..255
        img_BGRA[:,:,:3] = cloud_b                             # copy into the BGR part of img_BGRA
    else:
        # This uses img_f created above
        img_b = np.array(img_f*(255/8), dtype='B')   # bytes ranging 0..255
        img_BGRA[...,0] = img_b                      # copy range info onto Blue channel
        img_BGRA[...,1] = img_b                      # copy range info onto Green channel as well
        img_BGRA[...,2] = 0                          # leave red component 0, yielding cyan tone

    folded_BGRA[...] = thin_BGRA                     # broadcast the thin img_BGRA into the folded version of thick_BGRA
    display.paste(thick_BGRA)                        # paste the unfolded version of this onto the display

    # --- copy the cloud's point data to the visualization
    in_cloud = cloud * 0.98  # move the visualization slightly in to avoid colliding with seen objects
    in_cloud[in_cloud ==  np.inf] =  100  #  Move infinite values to large finite values
    in_cloud[in_cloud == -np.inf] = -100
    stale_time = robot.time - 0.9 / lidar.frequency  # move soon-to-be-replaced points out of sight, lest lidar see them
    fresh_time = robot.time - 0.1 / lidar.frequency  # update locations of fresh points
    for x in range(width):
        if lidar.cloud[0,x].time < stale_time:
            in_cloud[:,x,2] = -100  # hide deep underground
            refresh_this_column = True
        elif lidar.cloud[0,x].time > fresh_time:
            refresh_this_column = True
        else:
            refresh_this_column = False
        if refresh_this_column:
            for y in range(height):
                point_field[x + y*height] = in_cloud[y,x]

    point_field[:] = in_cloud.reshape(-1, 3)


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
        if max(right) > max(left):
            motors.velocity = SPEED, SPEED/2
        else:
            motors.velocity = SPEED/2, SPEED

    if print_readings: print(min_left, min_right)