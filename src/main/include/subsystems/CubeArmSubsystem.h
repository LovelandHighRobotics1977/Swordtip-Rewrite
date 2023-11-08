#include "headers/Headers.h"
#include "subsystems/ArmComponents/Angle.h"
#include "subsystems/ArmComponents/Intake.h"

#pragma once

class CubeArmSubsystem : public frc2::SubsystemBase {
	public:
		CubeArmSubsystem();

		void setTarget(int preset);

        void setIntake(bool enable, bool intake);

		frc2::StartEndCommand ShootCube();
		frc2::StartEndCommand PickupCube();
		frc2::InstantCommand StopIntake();

		frc2::FunctionalCommand RaiseArm();
		frc2::FunctionalCommand LowerArm();

	private:

		Angle m_angle;
        Intake m_intake;

        double motorSpeed;
	
};