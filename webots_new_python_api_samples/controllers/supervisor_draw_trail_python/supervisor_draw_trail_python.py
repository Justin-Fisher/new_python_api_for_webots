"""This is a "new Python API" implementation of the included Webots sample supervisor_draw_trail, which was in C.
   This sample illustrates using supervisor functionality to create and continually modify an IndexedLineSet
   to create a green trail of a given length behind a target node.
   This python version illustrates use of the new Python API's `world` module to access and modify parts of the
   scene tree, including using `world.plan` to dynamically plan new nodes to import into the tree.
   It also employs a somewhat simpler and more compact strategy for keeping track of the trail as older parts of it
   expire and new parts are created, re-using resources within the IndexedLine set. See explanatory comments inline."""

# /*
#  * Copyright Justin Fisher.
#  *
#  * Licensed under the Apache License, Version 2.0 (the "License");
#  * you may not use this file except in compliance with the License.
#  * You may obtain a copy of the License at
#  *
#  *     http://www.apache.org/licenses/LICENSE-2.0
#  *
#  * Unless required by applicable law or agreed to in writing, software
#  * distributed under the License is distributed on an "AS IS" BASIS,
#  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  * See the License for the specific language governing permissions and
#  * limitations under the License.
#  */

import world

TRAIL_LENGTH = 30       # Maximum number of points that will be included in trail
TRAIL_COLOR = (0, 1, 0) # RGB color to draw the trail in
REFRESH_FACTOR = 10     # Refresh the trail every REFRESH_FACTOR * WorldInfo.basicTimeStep.

# Get the target node whose trail we'll draw, the TARGET Transform in the E-puck turretSlot field.
target = world.TARGET
initial_pos = target.position  # the position of the target in global coordinates

# How often should we add a new point to the trail?
refresh_period = world.timestep_ms * REFRESH_FACTOR

# If any TRAIL_LINE_SET already exists in the world, silently remove it and its parent Shape
while world.Node("TRAIL_LINE_SET"):
    world.TRAIL_LINE_SET.parent.remove()

# Create a new TRAIL_LINE_SET (and a containing Shape and associated Appearance)
plan = world.plan  # for easier repeated reference
trail_plan = plan.IndexedLineSet(DEF = "TRAIL_LINE_SET",
                                 material = plan.Material(diffuseColor=TRAIL_COLOR, emissiveColor=TRAIL_COLOR),
                                 coord = plan.Coordinate(point=[initial_pos] * (TRAIL_LENGTH+1)),
                                 coordIndex = [0] * (TRAIL_LENGTH+2)
                                )
world.children.append(trail_plan)

# Store convenient references to relevant TRAIL_LINE_SET fields.
point_field = world.TRAIL_LINE_SET.coord.point  # list of 3D coordinates of points along the trail
index_field = world.TRAIL_LINE_SET.coordIndex   # list of successive indices of those points to connect, broken by -1

# If TRAIL_LENGTH is 9, then there would be 10 slots in point_field (always including at least 1 unused "gap"),
# and 11 slots in index_field (including gap and duplicated start/end). It will often look something like this:
#    [0, 1, 2, 3, 4, -1, 6, 7, 8, 9, 0]
# In this sequence, 4 would be the most recent "head" of the trail, and 6 its "tail", with the gap being in place of 5.
# This index_field says to link together the chain of most recent indices (0-4) up to, but not including, the gap (5),
# and also to link together the chain of older indices after the gap, linking back to the 0th (6-0)
# This allows us to make minimal changes to the lists as newly added "head" points push the gap around the cycle.
# For long trail lengths, this runs much faster than a simple alternative approach that repeatedly appends and removes
# points from point_field, which apparently causes a lot of time-consuming internal shuffling of the list.

index = 0  # Will always indicate the next index to be added (i.e. the current index of the "gap")

# Main loop.
while world.step(refresh_period):
    point_field[index] = target.position    # Store the current position of the target as the next point in the trail
    index_field[index] = index              # Connect this new point to its predecessor
    if index==0: index_field[-1] = 0        # The first and last points in the cycle are also connected
    index = (index+1) % (TRAIL_LENGTH+1)    # Advance to next index, cycling back if necessary
    index_field[index] = -1                 # Create a temporary gap to disconnect the new point from old ones after it
    if index==0: index_field[-1] = -1       # A gap at the start of the cycle is mirrored at the end
