"""This is a "new Python API" re-implementation of Webots' included "how to" sample vision, which was written in C++.
   This requires both `opencv-python` and `numpy`, installable via `pip install opencv-python` and `pip install numpy`.
   This sample illustrates converting a camera image to a numpy array (with camera.array), using some `opencv` and
   `numpy` commands to blacken out the parts of this image that do not match a set of acceptable colors,
   and sending the resulting image back to a Display device.  This sample also demonstrates reading single key-presses
   (to toggle which filters are active), and keys being held down (to remote-control drive the robot).
   And it demonstrates the use of world.Label to project information into the simulation window."""

# /*
 # * Copyright Justin Fisher.
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
 # 

import cv2 as cv    # opencv-python must be installed
import numpy as np  # numpy must be installed
import robot
import world

# === Initialize Devices ===

camera = robot.Camera("camera")  # this automatically enables the camera too

display = robot.Display("proc_im_display")

keyboard = robot.keyboard  # give keyboard local name for easy reference; this also automatically enables it

left_motor, right_motor = list(robot.Motor)
left_motor.target_position = right_motor.target_position = None  # we'll control these by velocity, below

# === Define Filters ===

# Minimum HSV values that count as each color (e.g. red must have hue>0, saturation>150, and value>30)
FILTER_MIN = dict(RED = (0, 150, 30),
                  GREEN = (58, 150, 30),
                  BLUE = (115, 150, 30),
                  YELLOW = (28, 150, 30),
                  PURPLE = (148, 150, 30),
                  WHITE = (0, 0, 170) )

# Maximum HSV values that count as each color (e.g. red must have hue<5 and has no upper limit on saturation and value)
FILTER_MAX = dict( RED = (5, 255, 255),
                   GREEN = (62, 255, 255),
                   BLUE = (120, 255, 255),
                   YELLOW = (32, 255, 255),
                   PURPLE = (152, 255, 255),
                   WHITE = (255, 80, 255) )

FILTERS = list(FILTER_MIN)               # ['RED', 'GREEN', ... ] for convenience
FILTER_KEYS = {f[0]:f for f in FILTERS}  # maps R to RED, G to GREEN, ...
# Beware: If you add a color with the same initial letter as another color, you'll need to assign FILTER_KEYS manually

active_filters = set(FILTERS)  # set of filters that are currently active; we start with all active

# === Prepare for Image Processing ===

# Create some arrays to store stages of processing; we'll re-use these to avoid proliferating arrays needlessly
bgr = np.zeros((camera.height, camera.width, 3), dtype=np.ubyte)              # input BGRA image without the A (alpha)
hsv = np.zeros((camera.height, camera.width, 3), dtype=np.ubyte)              # input converted to HSV
single_filter = np.zeros( (camera.height, camera.width, 1), dtype=np.ubyte)   # pixels that match a single color filter
combined_filter = np.zeros((camera.height, camera.width, 1), dtype=np.ubyte)  # pixels that match at least one filter
output = np.zeros((camera.height, camera.width, 4), dtype=np.ubyte)           # like input, but non-matching pixels black

# We'll also want to be able to view combined_filter as having thin 3rd dimension, so it can broadcast with input/output
combined_filter3D = combined_filter.reshape(camera.height, camera.width, 1)  # views the same data as combined_filter

def process_image(camera: robot.Camera, active_filters):
    """Returns an image like the one from the given camera, except blackens any pixel that doesn't match any color 
       filter.  If there are no filters, simply returns the camera input as is."""
    if not active_filters:
        return camera.array  # If there are no filters to worry about, simply return the camera's own image

    input = camera.array                        # numpy array sharing memory with the current Webots camera BGRA image
    cv.cvtColor(input, cv.COLOR_BGRA2BGR, bgr)  # make opencv drop the A (alpha) channel, store in bgr
    cv.cvtColor(bgr,   cv.COLOR_BGR2HSV,  hsv)  # make opencv convert to hsv (hue/saturation/value), store in hsv

    combined_filter.fill(0)  # Reset the combined_filter to a 2D array of just zeros (would blacken everything)
    for f in active_filters:
        # We add into the combined_filter every pixel that is close enough to the color of some filter f
        # We have opencv store each search result in single_filter to avoid needlessly creating extra arrays
        np.bitwise_or(combined_filter, cv.inRange(hsv, FILTER_MIN[f], FILTER_MAX[f], single_filter), combined_filter)

    return np.bitwise_and(input, combined_filter3D, output)  # black out pixels that match no color; store in output

# === Create Labels to show info in the Simulation Window ===

# Use world/supervisor to create an on-screen label to show which filters are active
filter_label = world.Label(pos=(0,0.8), size=0.16, color=0)
keys_label = world.Label(f"Alter with keys: {' '.join(FILTER_KEYS)} N A    Drive with arrows",
                         pos=(0.05,0.9), size=0.12, color=0)

def update_filter_label(active_filters):
    filter_label.update('Active filters: {' + ', '.join(active_filters) + '}')
    print(filter_label)  # print a copy of this label's text to the console as well

# === Print instructions into console ===

print("Vision module demo, using openCV and NumPy.")
print("Press arrow keys to move.")
for f, key in zip(FILTERS, FILTER_KEYS):
    print(f"Press {key} to apply/remove a {f.lower()} filter.")
print("Press A to apply all filters.")
print("Press N to remove all filters.")
print("When one or more filters is applied, only the matching colors are included in the image.")
print("The processed image consists of the entire image if no filter is used.")

# === Main loop ===

while robot.step():
    # --- Key-presses toggle filters ---
    for key in keyboard.pressed:  # includes only the keys that have just become pressed
        if key == 'N':
            active_filters = set()
        elif key == 'A':
            active_filters = set(FILTERS)
        elif key in FILTER_KEYS:
            active_filters ^= {FILTER_KEYS[key]}  # toggle (XOR) whether this filter is active
        if key in ['N','A'] or key in FILTER_KEYS: # if any of the above was hit, update our label of active filters
            update_filter_label(active_filters)

    # --- Arrow keys drive the robot ---
    speed_left = speed_right = 0
    # keyboard itself contains all keys that are currently down; we'll keep driving as long as arrow keys are down
    if keyboard.UP in keyboard:
        speed_left += 4.2
        speed_right += 4.2
    if keyboard.DOWN in keyboard:
        speed_left -= 4.2
        speed_right -= 4.2
    if keyboard.LEFT in keyboard:
        speed_left -= 1.2           # Turning any more sharply causes the e-puck to lose traction!
        speed_right += 1.2
    if keyboard.RIGHT in keyboard:
        speed_left += 1.2
        speed_right -= 1.2
    left_motor.velocity = speed_left
    right_motor.velocity = speed_right

    # --- Process and display camera image ---
    filtered_image = process_image(camera, active_filters)
    display.Image(filtered_image).paste_once(xy=(0, 0), blend=False)
    # Using .paste_once automatically deletes the imported image after pasting, so it won't clutter Webots memory
# end of main loop