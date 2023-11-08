#pragma once

#include "headers/Headers.h"

#include "subsystems/CubeArmSubsystem.h"

namespace ArmCommand {
    frc2::SequentialCommandGroup FireCube(CubeArmSubsystem *arm, int target);
    frc2::SequentialCommandGroup BeginCubePickup(CubeArmSubsystem *arm);
    frc2::SequentialCommandGroup EndCubePickup(CubeArmSubsystem *arm);
}