"""This is a "new Python API" re-implementation of Webots' included sample radar controller, which was written in C.
   This sample illustrates reading radar targets with `for target in radar` and consulting their features
   like `.distance` and `.azimuth`.  This also demonstrates use of `world` supervisor abilities, to create
   yellow spotlight cones and move them in the simulation to help indicate where the radar thinks targets are."""

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

import robot
from vectors import Pair  # convenient container for managing a left-right pair of similar devices, like Motors

SPEED = 6
TIME_STEP = 64
EXPECTED_NUMBER_OF_TARGETS = 3

# === Initialize Devices ===

display = robot.Display()  # When there is only one instance of a device class, providing its name is optional

keyboard = robot.keyboard  # give keyboard local name for easy reference; this also automatically enables it

motors = Pair(robot.Motor)     # Now motors.left will be the robot's first motor, motors.right the second
motors.target_position = None  # set both motors not to seek any target; we'll control them by velocity instead
motors.velocity = 0.0          # set both motors to be stopped, for now

ds = Pair(robot.DistanceSensor) # now we can address these as ds.left and ds.right

radar = robot.Radar()  # will be None if the robot has no radar; only the grey-capped robot has radar

# === Create spotlight markers to mark detected radar targets with ===
if radar is not None:
    import world  # so we can use supervisor powers to show where detected radar targets appear to be
    plan = world.plan  # for ease of reference
    # If the robot already has a RADAR_MARKERS group, empty it; otherwise create one
    if world.self.get_node('RADAR_MARKERS'):  # returns None if there is no such node
        world.self.RADAR_MARKERS.children.clear()
    else:
        world.self.children.append(plan.Group(DEF = "RADAR_MARKERS"))
    markers = world.self.RADAR_MARKERS.children  # the field in which the markers will live
    # We'll plan an inverted transparent glowing yellow cones to give a spotlight appearance on detected radar targets
    # The cones are invisibly attached to our robot, making frame-of-reference easy, but will jiggle as it does
    cone_color = (1,0.8,0)
    planned_cone = plan.Cone(height=0.8, bottomRadius=0.07,
                             baseColor=cone_color, emissiveColor = cone_color, transparency = 0.7,
                             translation=(0,0,-30), rotation=(1,0,0,3.1416))
    # Create as many cones as we expect to use; unused ones will spend their time hidden deep underground, z = -30
    for i in range(EXPECTED_NUMBER_OF_TARGETS):
        markers.append(planned_cone)

# === Main loop ===
while robot.step(TIME_STEP):
    # --- Main loop: Highlight radar targets (done only by the gray-capped robot) ---
    if radar is not None:
        for m in markers: m.translation = (0, 0, -30) # tentatively hide all radar markers deep below the ground
        # for each radar target, we'll move a yellow spotlight onto that target to highlight that we can see it
        for i, target in enumerate(radar):
            # Start with a vector pointing straight ahead the distance of the detected target
            pos = robot.Vector(target.distance,0,0)
            # setting a vector's .angle twists it to that new angle, keeping its magnitude the same
            pos.angle = -target.azimuth  # negative since this radar is mounted to give -azimuth in the +y direction
            markers[i].translation = pos  # store this pos as the new translation for this marker in the simulation
            # print( f"{robot.name} target #{i}: distance={target.distance}, azimuth={target.azimuth}")

    # --- Main loop: Avoid obstacles (done by all the robots) ---
    # The lookup tables on these distance sensors make them read 1024 for very close objects, and less for further ones,
    # so you could think of bigger numbers as louder proximity alert sirens!
    # If both distance sensors see something close, we're probably facing a wall, so we'll back away, turning left
    if all(ds > 500):
        motors.velocity = -SPEED , -SPEED / 2
    # Otherwise, if just one sees something close, we'll slow down and turn away
    #   (turning more sharply the closer that obstacle is, and also the more confined we are on the other side too)
    elif ds.right > 500:
        motors.velocity = -ds.right / 100, ds.left / 100 + 0.5
    elif ds.left > 500:
        motors.velocity = ds.right / 100 + 0.5, -ds.left / 100
    # Or if neither sees anything close, then we can zoom forward at maximal speed.
    else:
        motors.velocity = SPEED  # sets both .left and .right
