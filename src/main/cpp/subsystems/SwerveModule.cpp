// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"
#include "headers/Headers.h"

SwerveModule::SwerveModule(const int driveMotorID,     const int angleMotorID,       const int angleEncoderID, double magnetOffset)
					  : m_driveMotor{driveMotorID}, m_angleMotor{angleMotorID}, m_angleEncoder{angleEncoderID} {

	m_driveMotor.ConfigFactoryDefault();

	m_driveMotor.SetNeutralMode(NeutralMode::Brake);

	m_driveMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);

	m_driveMotor.Config_kP(0, Drivetrain::Swerve::PID::Motor::Drive::P);
	m_driveMotor.Config_kI(0, Drivetrain::Swerve::PID::Motor::Drive::I);
	m_driveMotor.Config_kD(0, Drivetrain::Swerve::PID::Motor::Drive::D);
	m_driveMotor.Config_kF(0, Drivetrain::Swerve::PID::Motor::Drive::F);

	m_driveMotor.ConfigNominalOutputForward(0);
	m_driveMotor.ConfigNominalOutputReverse(0);
	m_driveMotor.ConfigPeakOutputForward(1);
	m_driveMotor.ConfigPeakOutputReverse(-1);

	m_angleMotor.ConfigFactoryDefault();

	m_angleMotor.SetSensorPhase(true);
	m_angleMotor.SetNeutralMode(NeutralMode::Brake);

	m_angleMotor.ConfigFeedbackNotContinuous(true);
	m_angleMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0);
	m_angleMotor.ConfigRemoteFeedbackFilter(angleEncoderID, RemoteSensorSource(13), 0, 0);
	m_angleMotor.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Unsigned_0_to_360);

	m_angleMotor.Config_kP(0, Drivetrain::Swerve::PID::Motor::Angle::P);
	m_angleMotor.Config_kI(0, Drivetrain::Swerve::PID::Motor::Angle::I);
	m_angleMotor.Config_kD(0, Drivetrain::Swerve::PID::Motor::Angle::D);
	m_angleMotor.Config_kF(0, Drivetrain::Swerve::PID::Motor::Angle::F);
	m_angleMotor.Config_IntegralZone(0, 20);

	m_angleMotor.ConfigNominalOutputForward(0);
	m_angleMotor.ConfigNominalOutputReverse(0);
	m_angleMotor.ConfigPeakOutputForward(1);
	m_angleMotor.ConfigPeakOutputReverse(-1);

	m_angleEncoder.ConfigFactoryDefault();
	m_angleEncoder.ConfigMagnetOffset(magnetOffset);
	m_angleEncoder.SetPositionToAbsolute();
	m_angleEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
	
	m_turningPIDController.EnableContinuousInput(0_deg, 360_deg);
}

/*
frc::SwerveModuleState SwerveModule::GetState() {
	auto speed = units::meters_per_second_t{m_driveMotor.GetSelectedSensorVelocity() * Drivetrain::Swerve::Motor::Drive::distance_per_pulse.value()};
	auto angle = frc::Rotation2d{units::degree_t{m_angleEncoder.GetAbsolutePosition()}};
	return {speed * 10, angle};
}
*/

frc::SwerveModulePosition SwerveModule::GetPosition() {
	return {units::meter_t{m_driveMotor.GetSelectedSensorPosition() * Drivetrain::Swerve::Motor::Drive::distance_per_pulse},
			units::degree_t{m_angleEncoder.GetAbsolutePosition()}};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState, bool usePID) {
	// Optimize the reference state to avoid spinning further than 90 degrees
	state = frc::SwerveModuleState::Optimize(referenceState, units::degree_t{m_angleEncoder.GetAbsolutePosition()});

	// Calculate the drive output from the drive PID controller.
	driveOutput = m_drivePIDController.Calculate((m_driveMotor.GetSelectedSensorVelocity() * Drivetrain::Swerve::Motor::Drive::distance_per_pulse.value()), state.speed.value());

	// Calculate the turning motor output from the turning PID controller.
	angleOutput = m_turningPIDController.Calculate(units::degree_t{m_angleEncoder.GetAbsolutePosition()}, state.angle.Degrees());
	
	switch(m_angleEncoder.GetDeviceNumber()){
		case 2:
			frc::SmartDashboard::PutNumber("FR Raw Angle", -state.angle.Degrees().value());
			frc::SmartDashboard::PutNumber("FR Actual Angle", m_angleEncoder.GetAbsolutePosition());
			frc::SmartDashboard::PutNumber("FR PID Angle", angleOutput);
			frc::SmartDashboard::PutNumber("FR Velocity", driveOutput);
			break;
		case 5:
			frc::SmartDashboard::PutNumber("RR Raw Angle", -state.angle.Degrees().value());
			frc::SmartDashboard::PutNumber("RR Actual Angle", m_angleEncoder.GetAbsolutePosition());
			frc::SmartDashboard::PutNumber("RR PID Angle", angleOutput);
			frc::SmartDashboard::PutNumber("RR Velocity", driveOutput);
			break;
		case 8:
			frc::SmartDashboard::PutNumber("RL Raw Angle", -state.angle.Degrees().value());
			frc::SmartDashboard::PutNumber("RL Actual Angle", m_angleEncoder.GetAbsolutePosition());
			frc::SmartDashboard::PutNumber("RL PID Angle", angleOutput);
			frc::SmartDashboard::PutNumber("RL Velocity", driveOutput);
			break;
		case 11:
			frc::SmartDashboard::PutNumber("FL Raw Angle", -state.angle.Degrees().value());
			frc::SmartDashboard::PutNumber("FL Actual Angle", m_angleEncoder.GetAbsolutePosition());
			frc::SmartDashboard::PutNumber("FL PID Angle", angleOutput);
			frc::SmartDashboard::PutNumber("FL Velocity", driveOutput);
			break;
	}

	// Set the motor outputs.
	m_driveMotor.Set(driveOutput);
	if(usePID){
		m_angleMotor.Set(-angleOutput);
	}else{
		m_angleMotor.Set(TalonFXControlMode::Position, -state.angle.Degrees().value() * (4096/360));
	}
	
}