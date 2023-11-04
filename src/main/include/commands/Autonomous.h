#pragma once

#include "headers/Headers.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/CubeArmSubsystem.h"

#include "commands/ArmCommands.h"
#include "commands/SwerveCommands.h"

namespace AutoRoutine {
    frc2::CommandPtr autoOne(DriveSubsystem *drive, CubeArmSubsystem *arm);
}
