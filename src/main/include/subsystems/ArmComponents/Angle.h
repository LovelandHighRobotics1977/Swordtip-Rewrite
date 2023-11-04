#include "headers/Headers.h"

#pragma once

class Angle {
	public:
		Angle();

		void adjustAngle(bool up, bool down);

		bool GetUpperSwitch();

		bool GetLowerSwitch();

	private:

		WPI_TalonFX m_angleMotor;

		frc::DigitalInput m_lowerSwitch;
		frc::DigitalInput m_upperSwitch;
	
};