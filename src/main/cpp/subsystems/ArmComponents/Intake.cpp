#include "subsystems/ArmComponents/Intake.h"

Intake::Intake() :m_intakeMotor{Mechanism::Intake::ID}{}

void Intake::setIntake(double speed){
	m_intakeMotor.Set(speed);
}
