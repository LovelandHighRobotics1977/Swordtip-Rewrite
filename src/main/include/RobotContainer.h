// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "headers/Headers.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/CubeArmSubsystem.h"
#include "commands/Autonomous.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
	RobotContainer();

	frc2::Command* GetAutonomousCommand();

 private:
	// Controllers for driving the robot
	Driver m_driver{Teleop::Controller::Ports::Driver};
	Operator m_operator{Teleop::Controller::Ports::Operator};

	// The robot's subsystems and commands are defined here...

	// The robot's subsystems
	DriveSubsystem m_drive;
	CubeArmSubsystem m_CubeArm;

	// Auto routines
	frc2::CommandPtr m_AutoOne = AutoRoutine::autoOne(&m_drive, &m_CubeArm);

	// The chooser for the autonomous routines
	frc::SendableChooser<frc2::Command*> m_chooser;

	void ConfigureButtonBindings();
};
