// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "headers/Headers.h"
#include "subsystems/DriveSubsystem.h"

RobotContainer::RobotContainer() {
	// Initialize all of your commands and subsystems here

	// Configure the button bindings
	ConfigureButtonBindings();

	m_drive.SetDefaultCommand(frc2::ParallelCommandGroup(
		frc2::RunCommand( [this] { m_driver.update(); } ),
		frc2::RunCommand( [this] { m_operator.update(); } ),
		frc2::RunCommand( [this] { m_CubeArm.setTarget(m_operator.speed); } ),
		frc2::RunCommand( [this] { m_CubeArm.setIntake(m_operator.intakeEnable); } ),
		frc2::RunCommand( [this] { m_CubeArm.setAngle(m_operator.angle_up, m_operator.angle_down); } ),
		frc2::RunCommand( [this] { m_drive.Drive({
				m_driver.forward,
				m_driver.strafe,
				m_driver.rotate,
				m_driver.field_relative
			});},
			{&m_drive}
		)
	));
}

void RobotContainer::ConfigureButtonBindings() {

	// Configure your button bindings here

}

frc2::Command* RobotContainer::GetAutonomousCommand() {

	// Trajectory config
	frc::TrajectoryConfig config(Autonomous::Parameter::Linear::Velocity, Autonomous::Parameter::Linear::Acceleration);
	config.SetKinematics(m_drive.DriveKinematics);

	Trapezoid trapezoid{Autonomous::Controller::Proportional::Rotate, 0, 0, Autonomous::Controller::Constraint::Rotate};
	frc::ProfiledPIDController<units::radian> RotationController{ trapezoid.proportional, trapezoid.integral, trapezoid.derivative, trapezoid.constraint};
	RotationController.EnableContinuousInput(units::radian_t{-std::numbers::pi},units::radian_t{std::numbers::pi});

	// no auto
	return new frc2::SequentialCommandGroup(
		frc2::InstantCommand(
			[this] { m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); }
		),
		frc2::SwerveControllerCommand<4>(
			frc::TrajectoryGenerator::GenerateTrajectory(
				frc::Pose2d{0_m, 0_m, 0_deg},
				{
					frc::Translation2d{1_m, 1_m}, 
					frc::Translation2d{2_m, -1_m}
				},				
				frc::Pose2d{3_m, 0_m, 0_deg},
				config
			), 	
			[this]() { return m_drive.GetPose(); }, 
			m_drive.DriveKinematics,
			frc2::PIDController{Autonomous::Controller::Proportional::Forward, 0, 0},
			frc2::PIDController{Autonomous::Controller::Proportional::Strafe, 0, 0},
			RotationController,
			[this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },
			{&m_drive}
		),
		frc2::FunctionalCommand(
			[this] { m_CubeArm.setAngle(false, false); },
			[this] { m_CubeArm.setAngle(false, true); },
			[this] (bool interrupted) { m_CubeArm.setAngle(false, false); },
			[this] { return m_CubeArm.getLowerSwitch(); }
		),
		frc2::SwerveControllerCommand<4>(
			frc::TrajectoryGenerator::GenerateTrajectory(
				frc::Pose2d{3_m, 0_m, 0_deg},
				{
					frc::Translation2d{2_m, 1_m}, 
					frc::Translation2d{1_m, -1_m}
				},				
				frc::Pose2d{0_m, 0_m, 0_deg},
				config
			), 	
			[this]() { return m_drive.GetPose(); }, 
			m_drive.DriveKinematics,
			frc2::PIDController{Autonomous::Controller::Proportional::Forward, 0, 0},
			frc2::PIDController{Autonomous::Controller::Proportional::Strafe, 0, 0},
			RotationController,
			[this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },
			{&m_drive}
		),
		frc2::InstantCommand(
			[this]() { m_drive.Drive({}); }
		)
	);
}
