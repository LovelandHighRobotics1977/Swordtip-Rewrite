#include "subsystems/CubeArmSubsystem.h"

CubeArm::CubeArm(){}

void CubeArm::setTarget(int preset){
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

void CubeArm::setIntake(bool enable){
    if(enable){
        m_intake.setIntake(motorSpeed);
    }else{
        m_intake.setIntake(0);
    }
}

void CubeArm::setAngle(bool up, bool down){
    m_angle.adjustAngle(up, down);
}

bool CubeArm::getLowerSwitch(){
	return m_angle.GetLowerSwitch();
}

bool CubeArm::getUpperSwitch(){
	return m_angle.GetUpperSwitch();
}