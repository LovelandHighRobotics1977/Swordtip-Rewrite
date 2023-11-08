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

frc2::StartEndCommand CubeArmSubsystem::PickupCube(){
	return frc2::StartEndCommand( 
		[this] { m_intake.setIntake(0.2); } , 
		[this] { m_intake.setIntake(0); } , 
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