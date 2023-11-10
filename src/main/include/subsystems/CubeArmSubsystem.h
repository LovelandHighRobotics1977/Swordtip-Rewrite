#ifndef _SUBSYSTEM_CUBEARM_H
#define _SUBSYSTEM_CUBEARM_H

#include "headers/Headers.h"
#include "subsystems/ArmComponents/Angle.h"
#include "subsystems/ArmComponents/Intake.h"

class CubeArmSubsystem : public frc2::SubsystemBase {
	public:
		CubeArmSubsystem();

		void setTarget(int preset);

		frc2::StartEndCommand ShootCube();
		frc2::FunctionalCommand PickupCube();
		frc2::InstantCommand StopIntake();

		frc2::FunctionalCommand RaiseArm();
		frc2::FunctionalCommand LowerArm();

		double getTarget();

	private:

		Angle m_angle;
        Intake m_intake;

		PDH* m_PDH = PDH::GetInstance();

        double motorSpeed;
	
};

#endif // _SUBSYSTEM_CUBEARM_H