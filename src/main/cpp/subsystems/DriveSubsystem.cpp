// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"
#include "headers/Headers.h"
#include "commands/Autonomous.h"

DriveSubsystem::DriveSubsystem()
   :m_frontLeft{Drivetrain::Swerve::Module::Front::Left::Drive,
				Drivetrain::Swerve::Module::Front::Left::Angle,
				Drivetrain::Swerve::Module::Front::Left::Encoder,
				Drivetrain::Swerve::Module::Front::Left::MagnetOffset,},

	m_frontRight{Drivetrain::Swerve::Module::Front::Right::Drive,
				 Drivetrain::Swerve::Module::Front::Right::Angle,
				 Drivetrain::Swerve::Module::Front::Right::Encoder,
				 Drivetrain::Swerve::Module::Front::Right::MagnetOffset,},

	m_rearLeft{Drivetrain::Swerve::Module::Rear::Left::Drive,
			   Drivetrain::Swerve::Module::Rear::Left::Angle,
			   Drivetrain::Swerve::Module::Rear::Left::Encoder,
			   Drivetrain::Swerve::Module::Rear::Left::MagnetOffset,},

	m_rearRight{Drivetrain::Swerve::Module::Rear::Right::Drive,
				Drivetrain::Swerve::Module::Rear::Right::Angle,
				Drivetrain::Swerve::Module::Rear::Right::Encoder,
				Drivetrain::Swerve::Module::Rear::Right::MagnetOffset,},

	m_odometry{DriveKinematics,
				DriveSubsystem::GetHeading(),
				{m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
				m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
				frc::Pose2d{}} {}

void DriveSubsystem::Periodic() {
	OdometryData data;

	data.angle = DriveSubsystem::GetHeading();

	data.positions[0] = m_frontLeft.GetPosition();
	data.positions[1] = m_frontRight.GetPosition();
	data.positions[2] = m_rearLeft.GetPosition();
	data.positions[3] = m_rearRight.GetPosition();

	m_odometry.Update(data.angle, data.positions);
}

void DriveSubsystem::Drive(DriveData data) {

	fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(frc::ChassisSpeeds{data.forward, -data.strafe, data.rotate}, DriveSubsystem::GetHeading());
	robotRelativeSpeeds = frc::ChassisSpeeds{data.forward, -data.strafe, data.rotate};

	auto states = DriveKinematics.ToSwerveModuleStates(data.fieldRelative ? fieldRelativeSpeeds : robotRelativeSpeeds, data.centerOfRotation);

	DriveKinematics.DesaturateWheelSpeeds(&states, Teleop::Parameter::Linear::Velocity);

	auto [fl, fr, rl, rr] = states;
	
	m_frontLeft.SetDesiredState(fl);
	m_frontRight.SetDesiredState(fr);
	m_rearLeft.SetDesiredState(rl);
	m_rearRight.SetDesiredState(rr);

}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates) {

	DriveKinematics.DesaturateWheelSpeeds(&desiredStates, Autonomous::Parameter::Linear::Velocity);

	m_frontLeft.SetDesiredState(desiredStates[0],false);
	m_frontRight.SetDesiredState(desiredStates[1],false);
	m_rearLeft.SetDesiredState(desiredStates[2],false);
	m_rearRight.SetDesiredState(desiredStates[3],false);
}

frc::Rotation2d DriveSubsystem::GetHeading(){
	return -gyro->GetRotation2d();
}

frc2::CommandPtr DriveSubsystem::ZeroHeading() {
	return frc2::InstantCommand( [this] {gyro->Reset();} ).ToPtr();
}

frc2::SequentialCommandGroup DriveSubsystem::ZeroOdometry(frc::Pose2d pose) {
	return frc2::SequentialCommandGroup(
		frc2::InstantCommand( [this, pose] { DriveSubsystem::ResetOdometry(pose); } ),
		frc2::InstantCommand( [this] {gyro->Reset();} )
	);
}

double DriveSubsystem::GetTurnRate() {
	return -gyro->GetVelocityZ();
}

frc::Pose2d DriveSubsystem::GetPose() {
	return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
	OdometryData data;

	data.angle = gyro->GetRotation2d();

	data.positions[0] = m_frontLeft.GetPosition();
	data.positions[1] = m_frontRight.GetPosition();
	data.positions[2] = m_rearLeft.GetPosition();
	data.positions[3] = m_rearRight.GetPosition();

	m_odometry.ResetPosition(data.angle, data.positions, pose);
}
