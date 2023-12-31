// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "headers/Headers.h"
#include "subsystems/DriveSubsystem.h"

void RobotContainer::ConfigureAutonomousChooser() {

	m_chooser.SetDefaultOption("Only fire a cube", a_fireCubeOnly.get());

	switch (frc::DriverStation::Alliance()){
		case frc::DriverStation::kRed:
			m_chooser.AddOption("Drive Forward 3 Meters", a_red_driveForward.get());
			break;

		case frc::DriverStation::kBlue:

			break;

		case frc::DriverStation::kInvalid:
		
			break;
	}
}

void RobotContainer::ConfigureDashboard() {
	#define DASHBOARD frc::Shuffleboard::GetTab("Match Dashboard")

	DASHBOARD.Add(m_chooser);

	DASHBOARD.Add(PDH::GetInstance()->m_PDH);
	DASHBOARD.Add(Field2d::GetInstance()->m_field);

	DASHBOARD.AddDouble("CAN bus off", [] { return frc::RobotController::GetCANStatus().busOffCount; });
	DASHBOARD.AddDouble("CAN Faults",[] { return PDH::GetInstance()->CanWarnings(); });
	DASHBOARD.AddDouble("Brownouts",[] { return PDH::GetInstance()->Brownouts(); });
	DASHBOARD.AddDouble("Brownout Voltage", [] { return frc::RobotController::GetBrownoutVoltage().value(); });

	DASHBOARD.AddDouble("Battery Voltage", [] { return frc::DriverStation::GetBatteryVoltage();});
	DASHBOARD.AddBoolean("Replace Battery", [] { return frc::DriverStation::GetBatteryVoltage() < 12.6;});

	DASHBOARD.AddBoolean("Aim Low", [this] { return (m_CubeArm.getTarget() == 1) || (m_CubeArm.getTarget() == 4); });
	DASHBOARD.AddBoolean("Aim Mid", [this] { return (m_CubeArm.getTarget() == 2) || (m_CubeArm.getTarget() == 4); });
	DASHBOARD.AddBoolean("Aim High", [this] { return (m_CubeArm.getTarget() == 3) || (m_CubeArm.getTarget() == 4); });

	DASHBOARD.AddDouble("Throttle", [this] { return m_driver.throttle; });

	DASHBOARD.AddDouble("Heading", [] { return Gyro::GetInstance()->ahrs.GetYaw(); });

	DASHBOARD.AddDouble("X Pos", [this] { return m_drive.GetPose().X().value(); });
	DASHBOARD.AddDouble("Y Pos", [this] { return m_drive.GetPose().Y().value(); });
	DASHBOARD.AddDouble("Angle", [this] { return m_drive.GetHeading().Degrees().value(); });
}

void RobotContainer::ConfigureDefaultCommands() {
	m_driver.SetDefaultCommand(frc2::RunCommand( [this] { m_driver.update(); } , {&m_driver} ));
	m_operator.SetDefaultCommand(frc2::RunCommand( [this] { m_operator.update(); } , {&m_operator} ));
	m_CubeArm.SetDefaultCommand(frc2::RunCommand( [this] { m_CubeArm.setTarget(m_operator.speed); } , {&m_CubeArm} ));
	m_drive.SetDefaultCommand(frc2::RunCommand( 
		[this] { m_drive.Drive({ 
			m_driver.forward * Drivetrain::Movement::Maximum::Linear::Velocity, 
			m_driver.strafe * Drivetrain::Movement::Maximum::Linear::Velocity, 
			m_driver.rotate * Drivetrain::Movement::Maximum::Angular::Velocity, 
			m_driver.field_relative });}, 
		{&m_drive}
	));
}

void RobotContainer::ConfigureButtonBindings() {
	frc2::Trigger resetGyro([this] { return m_driver.gyro_reset; });
	resetGyro.OnTrue(frc2::InstantCommand( [] {Gyro::GetInstance()->ahrs.Reset();} ).ToPtr());

	frc2::Trigger bullshitMode([this] { return m_driver.coast_mode_toggle; });
	bullshitMode.OnTrue(frc2::InstantCommand( [this] { m_drive.SetNeutralMode(NeutralMode::Coast); }).ToPtr());
	bullshitMode.OnFalse(frc2::InstantCommand( [this] { m_drive.SetNeutralMode(NeutralMode::Brake); }).ToPtr());
	
	frc2::Trigger shootEnable([this] { return (m_operator.shootEnable); });
	shootEnable.WhileTrue(m_CubeArm.ShootCube().ToPtr());

	frc2::Trigger pickupEnable([this] { return (m_operator.pickupEnable); });
	pickupEnable.WhileTrue(m_CubeArm.PickupCube().ToPtr());

	frc2::Trigger raiseArm([this] { return m_operator.angle_up; });
	raiseArm.WhileTrue(m_CubeArm.RaiseArm().ToPtr());

	frc2::Trigger lowerArm([this] { return m_operator.angle_down; });
	lowerArm.WhileTrue(m_CubeArm.LowerArm().ToPtr());
}

RobotContainer::RobotContainer() {
	ConfigureAutonomousChooser();
	ConfigureDashboard();
	ConfigureButtonBindings();
	ConfigureDefaultCommands();
}

frc2::Command* RobotContainer::GetAutonomousCommand() { 
	return m_chooser.GetSelected(); 
}