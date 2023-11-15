// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef _SUBSYSTEM_SWERVE_MODULE_H
#define _SUBSYSTEM_SWERVE_MODULE_H

#include "headers/Headers.h"

class SwerveModule {
 public:
	SwerveModule(int driveMotorID, int angleMotorID, int angleEncoderID, double magnetOffset);

	frc::SwerveModuleState GetState();

	frc::SwerveModulePosition GetPosition();

	void SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode mode);

	void SetDesiredState(const frc::SwerveModuleState& state, bool usePID = true);

 private:
	WPI_TalonFX m_driveMotor;
	WPI_TalonFX m_angleMotor;
	CANCoder m_angleEncoder;

	frc::SwerveModuleState state;

	double driveOutput;
	double driveOutputPID;
	double angleOutput;

	frc2::PIDController m_drivePIDController{
		Drivetrain::Swerve::PID::Controller::Drive::P, 
		Drivetrain::Swerve::PID::Controller::Drive::I,
		Drivetrain::Swerve::PID::Controller::Drive::D
	};

	frc::ProfiledPIDController<units::degrees> m_turningPIDController{
		Drivetrain::Swerve::PID::Controller::Turning::P, 
		Drivetrain::Swerve::PID::Controller::Turning::I,
		Drivetrain::Swerve::PID::Controller::Turning::D,
		{Drivetrain::Movement::Maximum::Angular::Velocity*10, Drivetrain::Movement::Maximum::Angular::Acceleration*10}
	};
};

#endif	// _SUBSYSTEM_SWERVE_MODULE_H