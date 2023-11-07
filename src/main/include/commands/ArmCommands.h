#pragma once

#include "headers/Headers.h"

#include "subsystems/CubeArmSubsystem.h"

namespace ArmCommand {
    frc2::FunctionalCommand RaiseArm(CubeArmSubsystem *arm);
    frc2::FunctionalCommand LowerArm(CubeArmSubsystem *arm);
    frc2::SequentialCommandGroup FireCube(CubeArmSubsystem *arm, int target);
    frc2::InstantCommand EnableIntake(CubeArmSubsystem *arm);
    frc2::InstantCommand DisableIntake(CubeArmSubsystem *arm);
    frc2::SequentialCommandGroup BeginCubePickup(CubeArmSubsystem *arm);
    frc2::SequentialCommandGroup EndCubePickup(CubeArmSubsystem *arm);
}