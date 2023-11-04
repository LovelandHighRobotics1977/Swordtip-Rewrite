# Swordtip-Rewrite
Rewrite of the offseason swerve bot "Swordtip" using the command based framework.  This will be integrated into the 2024 season's bot as boilerplate code, and should only be used as a reference once Swordtip is decomissioned.  

Swordtip is a swerve bot using falcon 500 motors, with an arm that angles up and down, and a spinning wheel on the intake to pick up and fire cubes.  It can score low, mid, and high nodes.  It has a footprint of `26"`  x  `26"` without bumpers

# Most recent changes:
 - Auto has been moved to a separate file and is now selected with a sendable chooser
 - Multiple files for commands, each grouped into their respective namespaces.  
 - An auto that *should* be able to fetch a cube.  key word: *should*

# To do:
 - Mechanical:
     - Replace the front-right drive motor, because the bearing is failing.
     - Re-tread the swerve wheels, or potentially switch to the colsons
     - Re-work the intake so that the bottom has rollers, and so that the bolt is not necessary to stop the arm.  
 - Programming
     - Get the arm working fully as its own subsystem, its a bit goofy right now ( allow it to have a default command )
     - Tune speeds on the field to more accurate values for better autos
     - Add more auto routines to the list, teach Bradley and others how to use the new system
     - Auto balancing `( most likely not in 2024 season, so why bother? )`
 - Misc
     - SysId the robot for potential future ramesete controller usage?
     - Teach all programmers who have previously used timed how command works (bradley, dean, salem)

# Pipe dreams:
 - Vision processing
 - Assistive driving
 - VR integration through network tables?

---

With <3,

-Salem 

( 1977 Lead Programmer )
