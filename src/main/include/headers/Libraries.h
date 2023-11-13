// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Be wary all who enter, for beyond lies the wall of include statements

#ifndef _LIBRARIES_H
#define _LIBRARIES_H

// Standard Library
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <functional>
#include <utility>
#include <cmath>
#include <math.h>
#include <variant>

// Robot Program Framework
#include <frc/TimedRobot.h>

// CTRE Phoenix Hardware API
#include <ctre/Phoenix.h>

// REV Per-Device Hardware APIs
#include <rev/CANSparkMax.h>

// FRC PID Controller Libraries
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

// Hardware Interface Libraries
#include <frc/PowerDistribution.h>
#include <frc/RobotController.h>
#include <frc/DigitalInput.h>
#include <frc/RobotBase.h>
#include <frc/Encoder.h>

// Operator Input Libraries
#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>
#include <frc/Joystick.h>
#include <frc/GenericHID.h>
#include <frc/filter/SlewRateLimiter.h>

// Dashboard Libraries
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DriverStation.h>

// Vision Libraries
#include <cameraserver/CameraServer.h>

// Kinematics Libraries
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>

// Trajectory Libraries
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

// Geometry Libraries
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose2d.h>

// FRC2 Command Libraries
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/PIDCommand.h>

// FRC2 Trigger Libraries
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

// FRC2 Subsystem Libraries
#include <frc2/command/SubsystemBase.h>

// Units Libraries
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/time.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <units/math.h>
#include <units/dimensionless.h>

// Timing and Misc Libraries
#include <frc/Timer.h>

#endif  // _LIBRARIES_H