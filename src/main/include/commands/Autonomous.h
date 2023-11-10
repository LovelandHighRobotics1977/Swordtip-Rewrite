#pragma once

#include "headers/Headers.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/CubeArmSubsystem.h"

#include "commands/ArmCommands.h"
#include "commands/SwerveCommands.h"

namespace AutoRoutine {
    frc2::CommandPtr fireCubeOnly(DriveSubsystem *drive, CubeArmSubsystem *arm);

    namespace Red {
        frc2::CommandPtr MidPickupCube(DriveSubsystem *drive, CubeArmSubsystem *arm);
        frc2::CommandPtr DriveForward(DriveSubsystem *drive, CubeArmSubsystem *arm);
    }
    namespace Blue {

    }
}