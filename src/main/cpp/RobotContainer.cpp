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
	// Runs the chosen command in autonomous
	return m_chooser.GetSelected();
}
