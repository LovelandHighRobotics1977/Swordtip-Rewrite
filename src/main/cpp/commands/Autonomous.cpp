#include "commands/Autonomous.h"


frc2::CommandPtr AutoRoutine::autoOne(DriveSubsystem *drive, CubeArmSubsystem *arm) {
    return frc2::SequentialCommandGroup(
		SwerveCommand::ResetOdometry(drive, {0_m, 0_m, 0_deg}),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High),
		SwerveCommand::FollowPath(drive, 
			{
				frc::Translation2d{0.3_m, -1.9_m}
			}, 
			frc::Pose2d{1.4_m, -1.9_m, 0_deg}
		),
		ArmCommand::BeginCubePickup(arm),
		SwerveCommand::DriveForward(drive, 1_m),
		ArmCommand::EndCubePickup(arm),
		frc2::ParallelCommandGroup(
			ArmCommand::RaiseArm(arm),
			SwerveCommand::FollowPath(drive,
				{
					frc::Translation2d{0.3_m, -1.9_m}
				},
				frc::Pose2d{0_m, 0_m, 0_deg}
			)
		),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High)
	).ToPtr();
}