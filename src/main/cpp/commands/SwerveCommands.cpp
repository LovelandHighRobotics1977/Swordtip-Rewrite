#include "commands/SwerveCommands.h"

frc2::SwerveControllerCommand<4> SwerveCommand::FollowPath(DriveSubsystem *drive, frc::Pose2d startPose, std::vector<frc::Translation2d> waypoints, frc::Pose2d endPose) {
	
	frc::TrajectoryConfig config(Autonomous::Parameter::Linear::Velocity, Autonomous::Parameter::Linear::Acceleration);
	config.SetKinematics(drive->DriveKinematics);

	Trapezoid trapezoid{Autonomous::Controller::Proportional::Rotate, 0, 0, Autonomous::Controller::Constraint::Rotate};
	frc::ProfiledPIDController<units::radians> RotationController{ trapezoid.proportional, trapezoid.integral, trapezoid.derivative, trapezoid.constraint};
	RotationController.EnableContinuousInput(units::radian_t{-M_PI}, units::radian_t{M_PI});

	return frc2::SwerveControllerCommand<4>(
		frc::TrajectoryGenerator::GenerateTrajectory(
			startPose,
			waypoints,
			endPose,
			config
		), 	
		[drive]() { return drive->GetPose(); },
		drive->DriveKinematics,
		frc2::PIDController{Autonomous::Controller::Proportional::Forward, 0, 0},
		frc2::PIDController{Autonomous::Controller::Proportional::Strafe, 0, 0},
		RotationController,
		[drive](auto moduleStates) { drive->SetModuleStates(moduleStates); },
		{drive}
	);
}

frc2::FunctionalCommand SwerveCommand::DriveToPoint(DriveSubsystem *drive, frc::Pose2d desiredPoint) {
	
	auto difference_x = (desiredPoint.X() - drive->GetPose().X()).value();
	auto difference_y = (desiredPoint.Y() - drive->GetPose().Y()).value();
	
	auto magnitude = std::sqrt(difference_x * difference_x + difference_y * difference_y);

	auto forward = (magnitude != 0) ? std::max(-1.0, std::min(1.0, difference_x / magnitude)) : 0;
	auto strafe = (magnitude != 0) ? std::max(-1.0, std::min(1.0, difference_y / magnitude)) : 0;

	return frc2::FunctionalCommand(
		[drive] { drive->Drive({}); },
		[drive, forward, strafe] { drive->Drive({
			forward * Autonomous::Parameter::Linear::Velocity,
			strafe * Autonomous::Parameter::Linear::Velocity
		}); },
		[drive] (bool interrupted) { drive->Drive({}); },
		[drive, desiredPoint] { return drive->ComparePoses(desiredPoint, drive->GetPose(), 0.3_m); }
	);
}

frc2::InstantCommand SwerveCommand::ResetOdometry(DriveSubsystem *drive, frc::Pose2d pose) {
	return frc2::InstantCommand([drive, pose] { drive->ResetOdometry(pose); });
}
