# Swordtip-Rewrite
Rewrite of the offseason swerve bot "Swordtip" using the command based framework.  This will be integrated into the 2024 season's bot as boilerplate code, and should only be used as a reference once Swordtip is decomissioned.  

Swordtip is a swerve bot using falcon 500 motors, with an arm that angles up and down, and a spinning wheel on the intake to pick up and fire cubes.  It can score low, mid, and high nodes.  It has a footprint of `26"`  x  `26"` without bumpers

> [!IMPORTANT]  
> Any code previous to Version `2.0` __WILL NOT FUNCTION__ due to CAN ID changes

# Change Log:
 - Version `2.0`:
   - Added an implementation for getting data from the REV PDH
   - Added a current limiter to the Intake motor
   - Changed the way the arm is angled to suit operator preference.  
   - Changed the formatting of include guards to make it foolproof, and also potentially decrease compile times
   - Altered __EVERY__ CAN ID.  There are no exceptions, every CAN ID is different, and any previous robot code __WILL NOT WORK__
   - New dashboard implementation using Shuffleboard, with multiple tabs added for diagnostics
     - The dashboard save file is uploaded to the repository as "match.json"
   - Autonomous choosing is implemented, but there are no concrete routines.
  
 - Version `1.0`:
   - Auto has been moved to a separate file and is now selected with a sendable chooser
   - Multiple files for commands, each grouped into their respective namespaces.  
   - An auto that *should* be able to fetch a cube.  key word: *should*

# To do:
 - Mechanical:
   - Re-tread the swerve wheels, or potentially switch to the colsons
   - Re-work the intake so that the bottom has rollers, and so that the bolt is not necessary to stop the arm.
 - Electrical:
   - Check out the PDH so we don't have to be powering it via usb-c through the roborio
 - Programming
     - Tune linear speeds on the field
     - Tune angular speeds on the field
     - Add more auto routines to the list, teach Bradley and others how to use the new system
     - Auto balancing `( most likely not in 2024 season, so why bother? )`
     - Add "correct" starting positions based on april tags using the limelight
 - Misc
     - SysId the robot for potential future ramesete controller usage?
     - Teach all programmers who have previously used timed how command works (bradley, dean, salem)

# Pipe dreams:
 - Vision processing
 - Assistive driving
 - VR integration through network tables?

---

> [!NOTE]
> Programmed with <3 by Salem
>
> ( 1977 Lead Programmer )
