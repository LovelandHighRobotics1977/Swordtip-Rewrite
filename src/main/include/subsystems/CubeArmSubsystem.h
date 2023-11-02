#include "headers/Headers.h"
#include "subsystems/ArmComponents/Angle.h"
#include "subsystems/ArmComponents/Intake.h"

class CubeArm : public frc2::SubsystemBase {
	public:
		CubeArm();

		void setTarget(int preset);

        void setIntake(bool enable);

        void setAngle(bool up, bool down);
		bool getLowerSwitch();
		bool getUpperSwitch();

	private:

		Angle m_angle;
        Intake m_intake;

        double motorSpeed;
	
};