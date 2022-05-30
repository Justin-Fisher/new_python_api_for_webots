def main():  # Radon apparently only measures complexities for functions, so we wrap this all in a function
    import world

    TRAIL_LENGTH = 30       # Maximum number of points that will be included in trail
    TRAIL_COLOR = (0, 1, 0) # RGB color to draw the trail in
    REFRESH_FACTOR = 10     # Refresh the trail every REFRESH_FACTOR * WorldInfo.basicTimeStep.

    # Get the target node whose trail we'll draw, the TARGET Transform in the E-puck turretSlot field.
    target = world.TARGET
    initial_pos = target.position  # the position of the target in global coordinates

    # How often should we add a new point to the trail?
    refresh_period = world.timestep_ms * REFRESH_FACTOR

    # If any TRAIL already exists in the world, silently remove it and its parent Shape
    while world.Node("TRAIL"):
        world.TRAIL.parent.remove()

    # Create a new TRAIL (and a containing Shape and associated Appearance)
    plan = world.plan
    trail_plan = plan.IndexedLineSet(DEF = "TRAIL",
                                     material = plan.Material(diffuseColor=TRAIL_COLOR, emissiveColor=TRAIL_COLOR),
                                     coord = plan.Coordinate(point=[initial_pos] * (TRAIL_LENGTH+1)),
                                     coordIndex = [0] * (TRAIL_LENGTH+2)
                                    )
    world.children.append(trail_plan)

    # Store convenient references to relevant TRAIL fields.
    point_field = world.TRAIL.coord.point  # list of 3D coordinates of points along the trail
    index_field = world.TRAIL.coordIndex   # list of successive indices of those points to connect, broken by -1

    index = 0  # Will always indicate the next index to be added (i.e. the current index of the "gap")

    while world.step(refresh_period):
        point_field[index] = target.position    # Store the current position of the target as the next point in the trail
        index_field[index] = index              # Connect this new point to its predecessor
        if index==0: index_field[-1] = 0        # The first and last points in the cycle are also connected
        index = (index+1) % (TRAIL_LENGTH+1)    # Advance to next index, cycling back if necessary
        index_field[index] = -1                 # Create a temporary gap to disconnect the new point from old ones after it
        if index==0: index_field[-1] = -1       # A gap at the start of the cycle is mirrored at the end

if __name__ == "__main__":
    main()