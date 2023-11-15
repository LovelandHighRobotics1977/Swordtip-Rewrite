// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef _CONSTANTS_H
#define _CONSTANTS_H

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
				static const auto Radius = units::meter_t{std::hypot(Frame::Measurments::Length_Location.value(), Frame::Measurments::Width_Location.value()) / 39.375};
				static const auto Circumference = ((2*Radius) * M_PI);
			}
		}
	}
	namespace Swerve {
		namespace Module {
			namespace Front {
				namespace Left {
					static constexpr int Drive = 11;
					static constexpr int Angle = 12;
					static constexpr int Encoder = 13;
					static constexpr double MagnetOffset = 360 - 210.762;
					static constexpr frc::Translation2d Location = {+Frame::Measurments::Length_Location, +Frame::Measurments::Width_Location};
				}
				namespace Right {
					static constexpr int Drive = 2;
					static constexpr int Angle = 3;
					static constexpr int Encoder = 4;
					static constexpr double MagnetOffset = 360 - 112.859;
					static constexpr frc::Translation2d Location = {+Frame::Measurments::Length_Location, -Frame::Measurments::Width_Location};
				}
			}
			namespace Rear {
				namespace Left {
					static constexpr int Drive = 8;
					static constexpr int Angle = 9;
					static constexpr int Encoder = 10;
					static constexpr double MagnetOffset = 360 - 250.609;
					static constexpr frc::Translation2d Location = {-Frame::Measurments::Length_Location, +Frame::Measurments::Width_Location};
					}
				namespace Right {
					static constexpr int Drive = 5;
					static constexpr int Angle = 6;
					static constexpr int Encoder = 7;
					static constexpr double MagnetOffset = 360 - 258.398;
					static constexpr frc::Translation2d Location = {-Frame::Measurments::Length_Location, -Frame::Measurments::Width_Location};
				}
			}
		}
		namespace Wheel {
			static constexpr units::meter_t Radius = 2_in;
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
				static const auto distance_per_pulse = (((2 * Wheel::Radius) * M_PI) / (gear_ratio * encoder_cpr));
			}
			namespace Angle {
				static constexpr double max_rpm = 6380;
				static constexpr double gear_ratio = 150/7;
				static constexpr double encoder_cpr = 4096;
				static constexpr auto distance_per_pulse = units::radian_t{((2 * M_PI) / (encoder_cpr))};
			}
		}
	}
	namespace Movement {
		namespace Maximum {
			namespace Linear {
				// ~ 16 feet per second | ~ 5.02 meters per second
				static const auto Velocity = units::meters_per_second_t{((((2 * Swerve::Wheel::Radius.value() * M_PI) * (Swerve::Motor::Drive::max_rpm / Swerve::Motor::Drive::gear_ratio)))) / 60};
				static const auto Acceleration = units::meters_per_second_squared_t{Velocity.value()};
			}
			namespace Angular {
				// ~ 773 degrees per second
				static const auto Velocity = units::degrees_per_second_t{( 360 * ( Linear::Velocity / Frame::Measurments::Circle::Circumference ) ).value()};
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
			static constexpr int ID = 14;
		}
		namespace Limit {
			static constexpr int Lower = 0;
			static constexpr int Upper = 1;
		}
	}
	namespace Intake {
		static constexpr int ID = 15;
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

namespace Conversions {
	static constexpr auto DegToRad = M_PI / 180;
	static constexpr auto DegToSensorTicks = 4096 / 360;
}

namespace AutonomousMode {
    namespace Parameter {
		namespace Linear {
			static const auto Velocity = Drivetrain::Movement::Maximum::Linear::Velocity / 10;
			static const auto Acceleration = Drivetrain::Movement::Maximum::Linear::Acceleration / 10;
		}
		namespace Angular {
			static const auto Velocity = Drivetrain::Movement::Maximum::Angular::Velocity;
			static const auto Acceleration = Drivetrain::Movement::Maximum::Angular::Acceleration;
		}
	}
	namespace Controller {
		namespace Proportional {
			static constexpr double Forward = 0.1;
			static constexpr double Strafe = 0.1;
			static constexpr double Rotate = 0.1;
		}
		namespace Constraint {
			static const frc::TrapezoidProfile<units::radians>::Constraints Rotate{
				units::radians_per_second_t{Parameter::Angular::Velocity.value() * Conversions::DegToRad},
				units::radians_per_second_squared_t{Parameter::Angular::Acceleration.value() * Conversions::DegToRad}
			};
		}
	}
}

namespace TeleoperatedMode {
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

#endif // CONSTANTS_H