// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Be wary all who enter, for beyond lies the wall of include statements

#pragma once

// c++
#include <iostream>
#include <math.h>
#include <cmath>
#include <variant>
#include <vector>
#include <functional>
#include <numbers>
#include <utility>
#include <fstream>
#include <string>

// frc kinematics
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

// frc geometry
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>

// units
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

// ctre
#include <ctre/Phoenix.h>

// rev
#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/Spark.h>

// pid controllers
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

// encoders and gyros
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

// driver inputs
#include <frc/MathUtil.h> // for frc::ApplyDeadband
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>
#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/GenericHID.h>

// frc timed
#include <frc/TimedRobot.h>
#include <frc/Timer.h>

// dashboard
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/Shuffleboard.h>

// frc command based
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/FunctionalCommand.h>

// vision
#include <cameraserver/CameraServer.h>