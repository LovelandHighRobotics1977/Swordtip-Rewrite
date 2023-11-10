#ifndef _SUBSYSTEM_ARM_COMPONENTS_INTAKE_H
#define _SUBSYSTEM_ARM_COMPONENTS_INTAKE_H

#include "headers/Headers.h"

class Intake : public frc2::SubsystemBase {
	public:
		Intake();

		void setIntake(double speed);

	private:

		WPI_TalonFX m_intakeMotor;
	
};

#endif // _SUBSYSTEM_ARM_COMPONENTS_INTAKE_H