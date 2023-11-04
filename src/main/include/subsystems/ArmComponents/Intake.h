#include "headers/Headers.h"

#pragma once

class Intake {
	public:
		Intake();

		void setIntake(double speed);

	private:

		WPI_TalonFX m_intakeMotor;
	
};