#ifndef _SUBSYSTEM_ARM_COMPONENTS_ANGLE_H
#define _SUBSYSTEM_ARM_COMPONENTS_ANGLE_H

#include "headers/Headers.h"

class Angle : public frc2::SubsystemBase {
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

#endif // _SUBSYSTEM_ARM_COMPONENTS_ANGLE_H