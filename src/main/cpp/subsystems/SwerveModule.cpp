// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"
#include "headers/Headers.h"

SwerveModule::SwerveModule(const int driveMotorID,     const int angleMotorID,       const int angleEncoderID, double magnetOffset)
					  : m_driveMotor{driveMotorID}, m_angleMotor{angleMotorID}, m_angleEncoder{angleEncoderID} {

	Drivetrain::Swerve::Motor::Drive::Configure(&m_driveMotor);

	Drivetrain::Swerve::Motor::Angle::Configure(&m_angleMotor, angleEncoderID);

	Drivetrain::Swerve::Encoder::Angle::Configure(&m_angleEncoder, magnetOffset);
	
	m_turningPIDController.EnableContinuousInput(0_deg, 360_deg);
}

frc::SwerveModuleState SwerveModule::GetState() {
	auto speed = units::meters_per_second_t{m_driveMotor.GetSelectedSensorVelocity() * Drivetrain::Swerve::Motor::Drive::distance_per_pulse.value()};
	auto angle = frc::Rotation2d{units::degree_t{m_angleEncoder.GetAbsolutePosition()}};
	return {speed, angle};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
	return {units::meter_t{m_driveMotor.GetSelectedSensorPosition() * Drivetrain::Swerve::Motor::Drive::distance_per_pulse},
			units::degree_t{m_angleEncoder.GetAbsolutePosition()}};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
	// Optimize the reference state to avoid spinning further than 90 degrees
	state = frc::SwerveModuleState::Optimize(referenceState, units::degree_t{m_angleEncoder.GetAbsolutePosition()});

	// Calculate the drive output from the drive PID controller.
	driveOutput = m_drivePIDController.Calculate(m_driveMotor.GetSelectedSensorVelocity()*Drivetrain::Swerve::Motor::Drive::distance_per_pulse.value(), state.speed.value());

	// Calculate the turning motor output from the turning PID controller.
	angleOutput = m_turningPIDController.Calculate(units::degree_t{m_angleEncoder.GetAbsolutePosition()}, state.angle.Degrees());
	
	// Set the motor outputs.
	m_driveMotor.Set(driveOutput);
	m_angleMotor.Set(-angleOutput);
}