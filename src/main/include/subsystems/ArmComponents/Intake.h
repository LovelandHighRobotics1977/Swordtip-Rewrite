#include "headers/Headers.h"

#pragma once

class Intake : public frc2::SubsystemBase {
	public:
		Intake();

		void setIntake(double speed);

	private:

		WPI_TalonFX m_intakeMotor;
	
};