def main():  # Radon apparently only measures complexities for functions, so we wrap this all in a function
    from controller import Supervisor
    world = Supervisor()

    TRAIL_LENGTH = 30       # Maximum number of points that will be included in trail
    TRAIL_COLOR = (0, 1, 0) # RGB color to draw the trail in
    REFRESH_FACTOR = 10     # Refresh the trail every REFRESH_FACTOR * WorldInfo.basicTimeStep.

    # Get the target node whose trail we'll draw, the TARGET Transform in the E-puck turretSlot field.
    target = world.getFromDef('TARGET')
    initial_pos = target.getPosition()  # the position of the target in global coordinates

    # How often should we add a new point to the trail?
    refresh_period = world.getBasicTimeStep() * REFRESH_FACTOR

    # If any TRAIL already exists in the world, silently remove it and its parent Shape
    while True:
        old_trail = world.getFromDef("TRAIL")
        if old_trail is None: break
        old_trail.getParent().remove()

    # Create a new TRAIL (and a containing Shape and associated Appearance)
    TRAIL_COLOR_STRING = ' '.join(str(c) for c in TRAIL_COLOR)
    initial_pos_string = ' '.join(str(c) for c in initial_pos) + ' '
    material_plan = "Material {" + "diffuseColor" + TRAIL_COLOR_STRING + "emissiveColor" + TRAIL_COLOR_STRING + " }"
    appearance_plan = "Appearance { material " + material_plan + ' }'
    coord_field_plan = " coord Coordinate " + ( initial_pos_string * (TRAIL_LENGTH+1))
    index_field_plan = " coordIndex " + ('0 ' * (TRAIL_LENGTH + 2))
    geometry_plan = "DEF TRAIL IndexedLineSet {" + coord_field_plan + index_field_plan + ' }'
    trail_plan = "Shape { appearance " + appearance_plan + " geometry " + geometry_plan + ' }'
    world.getField("children").importMFNode(-1, trail_plan)

    # Store convenient references to relevant TRAIL fields.
    trail_node = world.getFromDef("TRAIL")
    point_field = trail_node.getField("coord").getSFNode().getField("point") # list of 3D coordinates of points along the trail
    index_field = trail_node.getField("coordIndex")   # list of successive indices of those points to connect, broken by -1

    index = 0  # Will always indicate the next index to be added (i.e. the current index of the "gap")

    while world.step(refresh_period) != -1:
        point_field.setMFVec3f(index, target.getPosition())  # Store the current position of the target as the next point in the trail
        index_field.setMFint32(index, index)                 # Connect this new point to its predecessor
        if index==0: index_field.setMFint32(-1, 0)           # The first and last points in the cycle are also connected
        index = (index+1) % (TRAIL_LENGTH+1)                 # Advance to next index, cycling back if necessary
        index_field.setMFint32(index, -1)                    # Create a temporary gap to disconnect the new point from old ones after it
        if index==0: index_field.setMFint32(-1, -1)          # A gap at the start of the cycle is mirrored at the end

if __name__ == "__main__":
    main()