// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "headers/Headers.h"
#include "SwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:
	DriveSubsystem();

	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	void Periodic() override;

	// Subsystem methods go here.

	/**
	 * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
	 * and the linear speeds have no effect on the angular speed.
	 *
	 * @param forward        Speed of the robot in the x direction
	 *                      (forward/backwards).
	 * @param strafe        Speed of the robot in the y direction (sideways).
	 * @param rotate           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to
	 *                      the field.
	 * @param centerOfRotation Center of the robot's rotation ( translation 2d )
	 */
	void Drive(DriveData data);

	/**
	 * Sets the drive MotorControllers to a power from -1 to 1.
	 */
	void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from 180 to 180
	 */
	units::degree_t GetHeading() const;

	/**
	 * Zeroes the heading of the robot.
	 */
	void ZeroHeading();

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	double GetTurnRate();

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	frc::Pose2d GetPose();

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	void ResetOdometry(frc::Pose2d pose);

	frc::SwerveDriveKinematics<4> DriveKinematics{
		frc::Translation2d{Drivetrain::Swerve::Module::Front::Left::Location},
		frc::Translation2d{Drivetrain::Swerve::Module::Front::Right::Location},
		frc::Translation2d{Drivetrain::Swerve::Module::Rear::Left::Location},
		frc::Translation2d{Drivetrain::Swerve::Module::Rear::Right::Location}
		};

private:
	SwerveModule m_frontLeft;
	SwerveModule m_frontRight;
	SwerveModule m_rearLeft;
	SwerveModule m_rearRight;

	frc::ChassisSpeeds fieldRelativeSpeeds;
	frc::ChassisSpeeds robotRelativeSpeeds;

	Gyro* gyro = Gyro::GetInstance();

	frc::SwerveDriveOdometry<4> m_odometry;
};
