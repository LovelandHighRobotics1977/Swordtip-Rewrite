// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Libraries.h"
#include "Util.h"

namespace Drivetrain {
	namespace Frame {
		namespace Measurments {
			static constexpr units::inch_t Length = 26_in;  									//  |Front left| frame to |rear left| frame
			static constexpr units::inch_t Width = 26_in;  										//  |Front left| frame to |front right| frame

			static constexpr units::inch_t Length_Offset = 2.625_in;  							//  distance from edge of frame to wheel
			static constexpr units::inch_t Width_Offset = 2.625_in;  							//  distance from edge of frame to wheel

			static constexpr units::inch_t Length_Location = ((Length/2)-Length_Offset);		//	distance from center to wheel |Left / Right|
			static constexpr units::inch_t Width_Location = ((Width/2)-Width_Offset);			//	distance from center to wheel |Front / Back|

			// circumscribed circle around length and width location
			namespace Circle {
				static const auto Diameter = std::hypot(Frame::Measurments::Length_Location.value(), Frame::Measurments::Width_Location.value()) * 2;
				static const auto Circumference = units::meter_t{(Diameter * M_PI) / 39.375};
			}
		}
	}
	namespace Swerve {
		namespace Module {
			namespace Front {
				namespace Left {
					static constexpr int Drive = 9;
					static constexpr int Angle = 10;
					static constexpr int Encoder = 11;
					static constexpr double MagnetOffset = 149.6777;
					static constexpr frc::Translation2d Location = {+Frame::Measurments::Length_Location, +Frame::Measurments::Width_Location};
				}
				namespace Right {
					static constexpr int Drive = 0;
					static constexpr int Angle = 1;
					static constexpr int Encoder = 2;
					static constexpr double MagnetOffset = -112.588;
					static constexpr frc::Translation2d Location = {+Frame::Measurments::Length_Location, -Frame::Measurments::Width_Location};
				}
			}
			namespace Rear {
				namespace Left {
					static constexpr int Drive = 6;
					static constexpr int Angle = 7;
					static constexpr int Encoder = 8;
					static constexpr double MagnetOffset = 111.0059;
					static constexpr frc::Translation2d Location = {-Frame::Measurments::Length_Location, +Frame::Measurments::Width_Location};
					}
				namespace Right {
					static constexpr int Drive = 3;
					static constexpr int Angle = 4;
					static constexpr int Encoder = 5;
					static constexpr double MagnetOffset = 101.777;
					static constexpr frc::Translation2d Location = {-Frame::Measurments::Length_Location, -Frame::Measurments::Width_Location};
				}
			}
		}
		namespace Wheel {
			static constexpr units::inch_t Radius = 2_in;
		}
		namespace PID {
			namespace Motor {
				namespace Drive {
					static constexpr double P = 0.001;
					static constexpr double I = 0;
					static constexpr double D = 0.005;
					static constexpr double F = 1;
				}
				namespace Angle {
					static constexpr double P = 1.7;
					static constexpr double I = 0.0016;
					static constexpr double D = 160;
					static constexpr double F = 0;
				}
			}
			namespace Controller {
				namespace Drive {
					static constexpr double P = 1;
					static constexpr double I = 0;
					static constexpr double D = 0;
				}
				namespace Turning {
					static constexpr double P = 0.005;
					static constexpr double I = 0.001;
					static constexpr double D = 0;
				}
			}
		}
		namespace Motor {
			namespace Drive {
				static constexpr double max_rpm = 6380;
				static constexpr double gear_ratio = 6.75;
				static constexpr double encoder_cpr = 2048;
				static const auto distance_per_pulse = units::meter_t{(((((2 * Wheel::Radius) * M_PI) / (gear_ratio * encoder_cpr)) / 60 ) / 39.375 ).value()};
				static void Configure(ctre::phoenix::motorcontrol::can::WPI_TalonFX *motor){
					motor->ConfigFactoryDefault();

					motor->SetNeutralMode(NeutralMode::Brake);

					motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);

					motor->Config_kP(0, PID::Motor::Drive::P);
					motor->Config_kI(0, PID::Motor::Drive::I);
					motor->Config_kD(0, PID::Motor::Drive::D);
					motor->Config_kF(0, PID::Motor::Drive::F);

					motor->ConfigNominalOutputForward(0);
					motor->ConfigNominalOutputReverse(0);
					motor->ConfigPeakOutputForward(1);
					motor->ConfigPeakOutputReverse(-1);
				}
			}
			namespace Angle {
				static constexpr double max_rpm = 6380;
				static constexpr double gear_ratio = 150/7;
				static constexpr double encoder_cpr = 4096;
				static constexpr auto distance_per_pulse = ((2 * M_PI) / (encoder_cpr));
				static void Configure(ctre::phoenix::motorcontrol::can::WPI_TalonFX *motor, int angleEncoderID){
					motor->ConfigFactoryDefault();

					motor->SetSensorPhase(true);
					motor->SetNeutralMode(NeutralMode::Brake);

					motor->ConfigFeedbackNotContinuous(true);
					motor->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0);
					motor->ConfigRemoteFeedbackFilter(angleEncoderID, RemoteSensorSource(13), 0, 0);
					motor->ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Unsigned_0_to_360);

					motor->Config_kP(0, PID::Motor::Angle::P);
					motor->Config_kI(0, PID::Motor::Angle::I);
					motor->Config_kD(0, PID::Motor::Angle::D);
					motor->Config_kF(0, PID::Motor::Angle::F);
					motor->Config_IntegralZone(0, 20);

					motor->ConfigNominalOutputForward(0);
					motor->ConfigNominalOutputReverse(0);
					motor->ConfigPeakOutputForward(1);
					motor->ConfigPeakOutputReverse(-1);
				}
			}
		}
		namespace Encoder {
			namespace Angle {
				static void Configure(ctre::phoenix::sensors::CANCoder *encoder, double magnetOffset){
					encoder->ConfigFactoryDefault();
					encoder->ConfigMagnetOffset(magnetOffset);
					encoder->SetPositionToAbsolute();
					encoder->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
				}
			}
		}
	}
	namespace Movement {
		namespace Maximum {
			namespace Linear {
				// ~ 16 feet per second | ~ 5.02 meters per second
				static const auto Velocity = units::meters_per_second_t{((((2 * Swerve::Wheel::Radius * M_PI) * (Swerve::Motor::Drive::max_rpm / Swerve::Motor::Drive::gear_ratio)) / 60 ).value()) / 39.375};
				static const auto Acceleration = units::meters_per_second_squared_t{Velocity.value()};
			}
			namespace Angular {
				// ~ 773 degrees per second
				static const auto Velocity = units::degrees_per_second_t{( 360 * (Linear::Velocity / Frame::Measurments::Circle::Circumference)).value()};
				static const auto Acceleration = units::degrees_per_second_squared_t{Velocity.value()};
			}
		}
		namespace Rotate {
			namespace Preset {
				static const units::degrees_per_second_t None = 0_deg_per_s;  						//  0 degrees per second
				static const units::degrees_per_second_t Slow =  Maximum::Angular::Velocity / 3;	//  260 degrees per second
				static const units::degrees_per_second_t Medium = Maximum::Angular::Velocity / 2;	//  370 degrees per second
				static const units::degrees_per_second_t Fast = Maximum::Angular::Velocity;  		//  770 degrees per second
			}
			namespace Around {
				static constexpr frc::Translation2d Center = {0_in,0_in};   						//  position of the center of the robot
			}
		}
	}
}

namespace Mechanism {
	namespace Angle {
		namespace Motor {
			static constexpr int ID = 12;
		}
		namespace Limit {
			static constexpr int Lower = 0;
			static constexpr int Upper = 1;
		}
	}
	namespace Intake {
		static constexpr int ID = 13;
		enum Target {
			Pickup = -1,
			Off,
			Low,
			Mid,
			High,
			Launch
		};
	}
}

namespace Teleop {
	namespace Parameter {
		namespace Linear {
			// Max horizontal velocity of ~16 feet per second
			static const auto Velocity = Drivetrain::Movement::Maximum::Linear::Velocity;
		}
		namespace Angular {
			// Max rotational velocity of ~773 degrees per second
			static const auto Velocity = Drivetrain::Movement::Maximum::Angular::Velocity;
		}
	}
	namespace Controller {
		namespace Ports {
			static constexpr int Driver = 0;
			static constexpr int Operator = 1;
		}
	}
}

namespace Autonomous {
	namespace Parameter {
		namespace Linear {
			static const auto Velocity = Drivetrain::Movement::Maximum::Linear::Velocity;
			static const auto Acceleration = Drivetrain::Movement::Maximum::Linear::Acceleration;
		}
		namespace Angular {
			static const auto Velocity = DegPerS_to_RadPerS(Drivetrain::Movement::Maximum::Angular::Velocity);
			static const auto Acceleration = DegPerS2_to_RadPerS2(Drivetrain::Movement::Maximum::Angular::Acceleration);
		}
	}
	namespace Controller {
		namespace Proportional {
			constexpr double Forward = 0.1;
			constexpr double Strafe = 0.1;
			constexpr double Rotate = 0.1;
		}
		namespace Constraint {
			const frc::TrapezoidProfile<units::radians>::Constraints Rotate{Parameter::Angular::Velocity, Parameter::Angular::Acceleration};
		}
	}
}