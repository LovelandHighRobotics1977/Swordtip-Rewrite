#include "subsystems/ArmComponents/Intake.h"

Intake::Intake() :m_intakeMotor{Mechanism::Intake::ID}{
	m_intakeMotor.SetNeutralMode(Brake);
}

void Intake::setIntake(double speed){
	m_intakeMotor.Set(speed);
}
