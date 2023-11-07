// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "headers/Headers.h"
#include "subsystems/DriveSubsystem.h"

RobotContainer::RobotContainer() {
	// Add commands to the autonomous chooser
	m_chooser.SetDefaultOption("Auto 1", m_AutoOne.get());
	m_chooser.AddOption("Auto 2", m_AutoOne.get());

	// Add the chooser to the dashboard
	frc::Shuffleboard::GetTab("Autonomous").Add(m_chooser);

	// Configure the button bindings
	ConfigureButtonBindings();

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

	// Configure your button bindings here

	frc2::Trigger resetGyro([this] { return m_driver.gyro_reset; });
	resetGyro.OnTrue(m_drive.ZeroHeading());
	resetGyro.OnTrue(frc2::InstantCommand([this] {m_drive.ResetOdometry({0_m,0_m,0_deg}); } ).ToPtr());

	frc2::Trigger intakeEnable([this] { return m_operator.intakeEnable; });
	intakeEnable.OnTrue(frc2::RunCommand( [this] { m_CubeArm.setIntake(true); } ).ToPtr());
	intakeEnable.OnFalse(frc2::RunCommand( [this] { m_CubeArm.setIntake(false); } ).ToPtr());

	frc2::Trigger armAngle([this] { return (m_operator.angle_up || m_operator.angle_down); });
	armAngle.OnTrue(frc2::RunCommand( [this] { m_CubeArm.setAngle(m_operator.angle_up, m_operator.angle_down); } ).ToPtr());

}

frc2::Command* RobotContainer::GetAutonomousCommand() {

	return m_chooser.GetSelected();
}

