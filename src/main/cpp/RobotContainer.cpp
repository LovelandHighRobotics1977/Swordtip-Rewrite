// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "headers/Headers.h"
#include "subsystems/DriveSubsystem.h"

RobotContainer::RobotContainer() {
	// Add commands to the autonomous chooser

	m_chooser.SetDefaultOption("Only fire a cube", a_fireCubeOnly.get());
	switch (frc::DriverStation::Alliance()){
		case frc::DriverStation::kRed:
			m_chooser.AddOption("Drive Forward 3 Meters", a_red_driveForward.get());
			m_chooser.AddOption("Mid do shit", a_red_midPickupCube.get());
			break;

		case frc::DriverStation::kBlue:

			break;

		case frc::DriverStation::kInvalid:
		
			break;
	}

	// Add all values and choosers to the dashboard
	frc::Shuffleboard::GetTab("Match Dashboard").Add(m_chooser);

	frc::Shuffleboard::GetTab("Match Dashboard").Add(m_PDH->m_PDH);

	frc::Shuffleboard::GetTab("Match Dashboard").AddDouble("CAN bus off", [this] { return frc::RobotController::GetCANStatus().busOffCount; });
	frc::Shuffleboard::GetTab("Match Dashboard").AddDouble("CAN Faults",[this] { return m_PDH->CanWarnings(); });
	frc::Shuffleboard::GetTab("Match Dashboard").AddDouble("Brownouts",[this] { return m_PDH->Brownouts(); });
	frc::Shuffleboard::GetTab("Match Dashboard").AddDouble("Brownout Voltage", [this] { return frc::RobotController::GetBrownoutVoltage().value(); });

	frc::Shuffleboard::GetTab("Match Dashboard").AddDouble("Battery Voltage", [] { return frc::DriverStation::GetBatteryVoltage();});
	frc::Shuffleboard::GetTab("Match Dashboard").AddBoolean("Replace Battery", [] { return frc::DriverStation::GetBatteryVoltage() < 12.6;});

	frc::Shuffleboard::GetTab("Match Dashboard").AddBoolean("Aim Low", [this] { return (m_CubeArm.getTarget() == 1) || (m_CubeArm.getTarget() == 4); });
	frc::Shuffleboard::GetTab("Match Dashboard").AddBoolean("Aim Mid", [this] { return (m_CubeArm.getTarget() == 2) || (m_CubeArm.getTarget() == 4); });
	frc::Shuffleboard::GetTab("Match Dashboard").AddBoolean("Aim High", [this] { return (m_CubeArm.getTarget() == 3) || (m_CubeArm.getTarget() == 4); });

	frc::Shuffleboard::GetTab("Match Dashboard").AddDouble("Throttle", [this] { return m_driver.throttle; });

	frc::Shuffleboard::GetTab("Match Dashboard").Add(m_field);

	frc::Shuffleboard::GetTab("Match Dashboard").AddDouble("Heading", [this] { return -m_drive.GetHeading().Degrees().value(); });

	frc::Shuffleboard::GetTab("Match Dashboard").AddDouble("X Pos", [this] { return m_drive.GetPose().X().value(); });
	frc::Shuffleboard::GetTab("Match Dashboard").AddDouble("Y Pos", [this] { return m_drive.GetPose().Y().value(); });
	frc::Shuffleboard::GetTab("Match Dashboard").AddDouble("Angle", [this] { return m_drive.GetPose().Rotation().Degrees().value(); });

	// Configure the button bindings
	ConfigureButtonBindings();

	m_driver.SetDefaultCommand(frc2::RunCommand( [this] { m_driver.update(); } , {&m_driver} ));
	m_operator.SetDefaultCommand(frc2::RunCommand( [this] { m_operator.update(); } , {&m_operator} ));
	m_CubeArm.SetDefaultCommand(frc2::RunCommand( [this] { m_CubeArm.setTarget(m_operator.speed); } , {&m_CubeArm} ));
	m_drive.SetDefaultCommand(frc2::ParallelCommandGroup(
		frc2::RunCommand( 
			[this] { m_drive.Drive({ 
				m_driver.forward * Drivetrain::Movement::Maximum::Linear::Velocity, 
				m_driver.strafe * Drivetrain::Movement::Maximum::Linear::Velocity, 
				m_driver.rotate * Drivetrain::Movement::Maximum::Angular::Velocity, 
				m_driver.field_relative });}, 
			{&m_drive}
		),
		frc2::RunCommand([this]{ m_field.SetRobotPose(m_drive.GetPose()); })
	));
}

void RobotContainer::ConfigureButtonBindings() {

	// Configure your button bindings here

	frc2::Trigger resetGyro([this] { return m_driver.gyro_reset; });
	resetGyro.OnTrue(m_drive.ZeroHeading());
	
	frc2::Trigger shootEnable([this] { return (m_operator.shootEnable); });
	shootEnable.WhileTrue(m_CubeArm.ShootCube().ToPtr());

	frc2::Trigger pickupEnable([this] { return (m_operator.pickupEnable); });
	pickupEnable.WhileTrue(m_CubeArm.PickupCube().ToPtr());

	frc2::Trigger raiseArm([this] { return m_operator.angle_up; });
	raiseArm.WhileTrue(m_CubeArm.RaiseArm().ToPtr());

	frc2::Trigger lowerArm([this] { return m_operator.angle_down; });
	lowerArm.WhileTrue(m_CubeArm.LowerArm().ToPtr());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {

	return m_chooser.GetSelected();
}

