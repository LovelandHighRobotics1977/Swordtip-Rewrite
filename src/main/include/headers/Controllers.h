#pragma once

#include "Libraries.h"

class Driver {
	public:

		Driver(const int port):m_Joystick{port}{};

		bool field_relative;

		bool gyro_reset;

		bool trigger_one;
		bool trigger_two;

		bool emergency_stop;

		double throttle;

		units::meters_per_second_t forward;
		units::meters_per_second_t strafe;
		units::degrees_per_second_t rotate;
		
		/**
		 * Update the controller variables 
		 * @attention Each control scheme is defined in this function
		 * @note Automaically chooses the control scheme based on the joystick name
		*/
		void update(){
			if(m_Joystick.GetName() == std::string{"HOTAS"}){
				field_relative = !m_Joystick.GetRawButton(6);

				gyro_reset = m_Joystick.GetRawButton(2);

				trigger_one = m_Joystick.GetRawButton(1);
				trigger_two = m_Joystick.GetRawButton(15);

				emergency_stop = m_Joystick.GetRawButton(5);

				throttle = ((1 - ((m_Joystick.GetZ() + 1) / 2)));

				forward = (-m_forwardLimiter.Calculate(frc::ApplyDeadband(m_Joystick.GetY(), forward_deadzone)) * throttle) * Drivetrain::Movement::Maximum::Linear::Velocity;
				strafe = (-m_strafeLimiter.Calculate(frc::ApplyDeadband(m_Joystick.GetX(), strafe_deadzone)) * throttle) * Drivetrain::Movement::Maximum::Linear::Velocity;
				rotate = (-m_rotateLimiter.Calculate(frc::ApplyDeadband(m_Joystick.GetRawAxis(5), rotate_deadzone)) * throttle) * Drivetrain::Movement::Maximum::Angular::Velocity;
			}

			if(m_Joystick.GetName() == std::string{"Saitek X45"}){
				field_relative = !m_Joystick.GetRawButton(7);

				gyro_reset = m_Joystick.GetRawButton(4);

				trigger_one = m_Joystick.GetRawButton(1);
				trigger_two = m_Joystick.GetRawButton(1);

				emergency_stop = m_Joystick.GetRawButton(8);

				throttle = ((1 - ((m_Joystick.GetRawAxis(4) + 1) / 2)));

				forward = (-m_forwardLimiter.Calculate(frc::ApplyDeadband(m_Joystick.GetY(), forward_deadzone)) * throttle) * Drivetrain::Movement::Maximum::Linear::Velocity;
				strafe = (-m_strafeLimiter.Calculate(frc::ApplyDeadband(m_Joystick.GetX(), strafe_deadzone)) * throttle) * Drivetrain::Movement::Maximum::Linear::Velocity;
				rotate = (-m_rotateLimiter.Calculate(frc::ApplyDeadband(m_Joystick.GetRawAxis(3), rotate_deadzone)) * throttle) * Drivetrain::Movement::Maximum::Angular::Velocity;
			}

			if(m_Joystick.GetName() == std::string{"Extreme 3D pro"}){
				field_relative = !m_Joystick.GetRawButton(2);

				gyro_reset = m_Joystick.GetRawButton(3);

				trigger_one = m_Joystick.GetRawButton(1);
				trigger_two = m_Joystick.GetRawButton(1);

				emergency_stop = m_Joystick.GetRawButton(4);

				throttle = ((1 - ((m_Joystick.GetRawAxis(3) + 1) / 2)));

				forward = (-m_forwardLimiter.Calculate(frc::ApplyDeadband(m_Joystick.GetY(), forward_deadzone)) * throttle) * Drivetrain::Movement::Maximum::Linear::Velocity;
				strafe = (-m_strafeLimiter.Calculate(frc::ApplyDeadband(m_Joystick.GetX(), strafe_deadzone)) * throttle) * Drivetrain::Movement::Maximum::Linear::Velocity;
				rotate = (-m_rotateLimiter.Calculate(frc::ApplyDeadband(m_Joystick.GetRawAxis(2), rotate_deadzone)) * sqrt(throttle)) * Drivetrain::Movement::Maximum::Angular::Velocity;
			}
		
		}

		private:
			frc::Joystick m_Joystick;

			frc::SlewRateLimiter<units::dimensionless::scalar> m_forwardLimiter{3 / 1_s};
			frc::SlewRateLimiter<units::dimensionless::scalar> m_strafeLimiter{3 / 1_s};
			frc::SlewRateLimiter<units::dimensionless::scalar> m_rotateLimiter{3 / 1_s};

			double forward_deadzone = 0.1;
			double strafe_deadzone = 0.1;
			double rotate_deadzone = 0.3;
	};

class Operator {
	public:

		Operator(const int port):m_XboxController{port}{};

		bool angle_up;
		bool angle_down;

		bool intakeEnable;

		int speed;

		void update(){
			angle_up = m_XboxController.GetLeftTriggerAxis() > 0;
			angle_down = m_XboxController.GetRightTriggerAxis() > 0;

			intakeEnable = (m_XboxController.GetLeftBumper() || m_XboxController.GetRightBumper());

			if(m_XboxController.GetXButton() || m_XboxController.GetLeftBumper()){
				speed = Mechanism::Intake::Target::Pickup;
			}else if(m_XboxController.GetStartButton()){
				speed = Mechanism::Intake::Target::Off;
			}else if(m_XboxController.GetXButton()){
				speed = Mechanism::Intake::Target::Low;
			}else if(m_XboxController.GetYButton()){
				speed = Mechanism::Intake::Target::Mid;
			}else if(m_XboxController.GetBButton()){
				speed = Mechanism::Intake::Target::High;
			}else if(m_XboxController.GetAButton()){
				speed = Mechanism::Intake::Target::Launch;
			}
		}
	private:
		frc::XboxController m_XboxController;
	};