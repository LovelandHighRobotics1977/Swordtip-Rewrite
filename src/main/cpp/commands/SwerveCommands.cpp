#include "commands/SwerveCommands.h"

frc2::SequentialCommandGroup SwerveCommand::FollowPath(DriveSubsystem *drive, frc::Pose2d startPose, std::vector<frc::Translation2d> waypoints, frc::Pose2d endPose) {
	
	frc::TrajectoryConfig config(Autonomous::Parameter::Linear::Velocity, Autonomous::Parameter::Linear::Acceleration);
	config.SetKinematics(drive->DriveKinematics);

	Trapezoid trapezoid{Autonomous::Controller::Proportional::Rotate, 0, 0, Autonomous::Controller::Constraint::Rotate};
	frc::ProfiledPIDController<units::radians> RotationController{ trapezoid.proportional, trapezoid.integral, trapezoid.derivative, trapezoid.constraint};
	RotationController.EnableContinuousInput(units::radian_t{-M_PI}, units::radian_t{M_PI});

	auto Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
		startPose,
		waypoints,
		endPose,
		config
	);

	return frc2::SequentialCommandGroup(
		frc2::InstantCommand( [Trajectory] { Field2d::GetInstance()->m_field.GetObject("Trajectory")->SetTrajectory(Trajectory); } ),
		frc2::SwerveControllerCommand<4>(
			Trajectory,
			[drive]() { return drive->GetPose(); }, // BULLSHIT MAGIC NUMBER
			drive->DriveKinematics,
			frc2::PIDController{Autonomous::Controller::Proportional::Forward, 0, 0},
			frc2::PIDController{Autonomous::Controller::Proportional::Strafe, 0, 0},
			RotationController,
			[drive](auto moduleStates) { drive->SetModuleStates(moduleStates); },
			{drive}
		)
	);
}

frc2::InstantCommand SwerveCommand::ResetOdometry(DriveSubsystem *drive, frc::Pose2d pose) {
	return frc2::InstantCommand([drive, pose] { drive->ResetOdometry(pose); });
}
