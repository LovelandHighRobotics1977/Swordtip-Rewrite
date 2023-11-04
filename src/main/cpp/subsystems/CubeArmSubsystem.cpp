#include "subsystems/CubeArmSubsystem.h"

CubeArmSubsystem::CubeArmSubsystem(){}

void CubeArmSubsystem::setTarget(int preset){
    switch (preset){
		case Mechanism::Intake::Target::Pickup:
			motorSpeed = 0.2;
			break;
		case Mechanism::Intake::Target::Off:
			motorSpeed = 0;
			break;
		case Mechanism::Intake::Target::Low:
			motorSpeed = -0.2;
			break;
		case Mechanism::Intake::Target::Mid:
			motorSpeed = -0.4;
			break;
		case Mechanism::Intake::Target::High:
			motorSpeed = -0.9;
			break;
		case Mechanism::Intake::Target::Launch:
			motorSpeed = -1;
			break;
		default:
			break;
	}
}

void CubeArmSubsystem::setIntake(bool enable){
    if(enable){
        m_intake.setIntake(motorSpeed);
    }else{
        m_intake.setIntake(0);
    }
}

void CubeArmSubsystem::setAngle(bool up, bool down){
    m_angle.adjustAngle(up, down);
}

bool CubeArmSubsystem::getLowerSwitch(){
	return (!m_angle.GetLowerSwitch());
}

bool CubeArmSubsystem::getUpperSwitch(){
	return (!m_angle.GetUpperSwitch());
}