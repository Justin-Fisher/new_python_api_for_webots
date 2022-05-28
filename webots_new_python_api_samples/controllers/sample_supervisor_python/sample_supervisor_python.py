"""This is a "new Python API" implementation of Webots' included sample_supervisor, which was in C.
   This sample illustrates a variety of supervisor functionality.
   This python version illustrates use of the new Python API's `world` module to do the same things (in a much
   simpler and more intuitive way!), including using `world.plan` to plan a new nodes to import into the tree,
   and using `world.Label` to create and modify on-screen labels in a much more streamlined way."""

__author__ = "Justin C. Fisher"

#/*
# * Copyright 2022. Justin Fisher.
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
# */


import math
import world

# Print number of nodes in the root children field
print(f"This world contains {len(world.children)} nodes:")

# Print the types of the nodes that are present in the world
for node in world.children:
    print(f"-> {node.type}")
    
# Print the content of the 'gravity' field of the 'WorldInfo' node
print(f"WorldInfo.gravity = {world.WorldInfo.gravity}")

# Use a label to display information in the 3D view
label = world.Label("Going to move the location of the PointLight\n" +
                    "in 2 seconds (simulation time)...",
                    pos=(0, 0), size = 0.1, color=(0, 1, 0), transparency=0.1, font="Georgia")
print(label)      # Also print this same message to console
world.step(2000)  # Wait for 2000 milliseconds = 2 seconds
world.PointLight.location = (0.5, 0.5, 0.3)

# Import a new sphere node into the simulation after waiting 2 seconds
label.update("Going to import a Sphere in 2 seconds (simulation time)...")
print(label)  # also print this same message to console
world.step(2000)
planned_sphere = world.plan.Sphere(translation=(0,0,0), radius=0.1, subdivision=3, baseColor=(1,0,0))
world.children.append(planned_sphere)

# Main simulation loop
label.update("Going to move the Sphere in 2 seconds (simulation time)...")
print(label)  # also print this same message to console
world.step(2000)
label.delete()
timer = world.TimerLabel(duration = 60, pos=(0.4,0), size=0.2) # a timer that will count down 60 seconds
sphere_node = world.children[-1]
translation = sphere_node.translation
#  Main loop will continue so long as Webots' world.step() says to, and our 5-second timer has not run down
while world.step() and timer.update() > 0:  # keep going until 60 second timer runs out
    # move the Sphere node in a circle with 0.3m radius
    translation.x = 0.3 * math.cos(world.time)
    translation.y = 0.3 * math.sin(world.time)
    sphere_node.translation = translation
