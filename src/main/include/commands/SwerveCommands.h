#pragma once

#include "headers/Headers.h"

#include "subsystems/DriveSubsystem.h"

namespace SwerveCommand {
    frc2::SwerveControllerCommand<4> FollowPath(DriveSubsystem *drive, std::vector<frc::Translation2d> waypoints, frc::Pose2d endPose);
    frc2::SwerveControllerCommand<4> DriveForward(DriveSubsystem *drive, units::meter_t distance);
    frc2::InstantCommand ResetOdometry(DriveSubsystem *drive, frc::Pose2d pose);
}