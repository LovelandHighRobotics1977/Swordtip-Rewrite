#include "subsystems/ArmComponents/Angle.h"

Angle::Angle():
	m_angleMotor{Mechanism::Angle::Motor::ID},
	m_lowerSwitch{Mechanism::Angle::Limit::Lower},
	m_upperSwitch{Mechanism::Angle::Limit::Upper}{}

void Angle::adjustAngle(bool up, bool down){
	if(up && !down){
		if(m_upperSwitch.Get()){
			m_angleMotor.Set( 0.2 );
		}
	}else if(down && !up){
		if(m_lowerSwitch.Get()){
			m_angleMotor.Set( -0.2 );
		}
	}else{
		m_angleMotor.Set( 0 );
	}
}

bool Angle::GetUpperSwitch(){
	return m_upperSwitch.Get();
}

bool Angle::GetLowerSwitch(){
	return m_lowerSwitch.Get();
}