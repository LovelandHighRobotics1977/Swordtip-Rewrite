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

frc2::StartEndCommand CubeArmSubsystem::ShootCube(){
	return frc2::StartEndCommand(
		[this] { m_intake.setIntake(motorSpeed); } , 
		[this] { m_intake.setIntake(0); } , 
		{&m_intake}
	);
}

frc2::FunctionalCommand CubeArmSubsystem::PickupCube(){
	return frc2::FunctionalCommand( 
		[this] { m_intake.setIntake(0); } ,
		[this] { m_intake.setIntake(0.2); } , 
		[this] (bool interrupted) { m_intake.setIntake(0); } , 
		[this] { return (m_PDH->GetCurrent(14) > 7); },
		{&m_intake}
	);
}

frc2::InstantCommand CubeArmSubsystem::StopIntake(){
	return frc2::InstantCommand( 
		[this] { m_intake.setIntake(0); } , 
		{&m_intake}
	);
}

frc2::FunctionalCommand CubeArmSubsystem::RaiseArm(){
	return frc2::FunctionalCommand(
		[this] {m_angle.adjustAngle(0, 0); },
		[this] {m_angle.adjustAngle(true, 0); },
		[this] (bool interrupted) {m_angle.adjustAngle(0, 0); },
		[this] { return (!m_angle.GetUpperSwitch()); },
		{&m_angle}
	);
}

frc2::FunctionalCommand CubeArmSubsystem::LowerArm(){
	return frc2::FunctionalCommand(
		[this] {m_angle.adjustAngle(0, 0); },
		[this] {m_angle.adjustAngle(0, true); },
		[this] (bool interrupted) {m_angle.adjustAngle(0, 0); },
		[this] { return (!m_angle.GetLowerSwitch()); },
		{&m_angle}
	);
}

double CubeArmSubsystem::getTarget(){
	if(motorSpeed == -0.2){
		return 1;
	}else if(motorSpeed == -0.4){
		return 2;
	}else if(motorSpeed == -0.9){
		return 3;
	}else if(motorSpeed == -1){
		return 4;
	}else{
		return 0;
	}
};