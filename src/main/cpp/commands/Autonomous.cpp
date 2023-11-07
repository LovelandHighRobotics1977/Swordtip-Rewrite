#include "commands/Autonomous.h"

frc2::CommandPtr AutoRoutine::autoOne(DriveSubsystem *drive, CubeArmSubsystem *arm) {
    return frc2::SequentialCommandGroup(
		SwerveCommand::ResetOdometry(drive, {0_m, 0_m, 0_deg}),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High),
		SwerveCommand::FollowPath(drive, 
			{0_m, 0_m, 0_deg},
			{
				{0.3_m, -1.9_m}
			}, 
			{1.4_m, -1.9_m, 0_deg}
		),
		frc2::WaitCommand(0.5_s),
		ArmCommand::BeginCubePickup(arm),
		SwerveCommand::DriveToPoint(drive, {2_m, -1.9_m, 0_deg}),
		ArmCommand::EndCubePickup(arm),
		frc2::WaitCommand(0.5_s),
		frc2::ParallelCommandGroup(
			ArmCommand::RaiseArm(arm),
			frc2::SequentialCommandGroup(
				SwerveCommand::DriveToPoint(drive, {1.4_m, -1.9_m, 0_deg}),
				SwerveCommand::FollowPath(drive,
					{1.4_m, -1.9_m, 0_deg},
					{
						{0.3_m, -1.9_m}
					},
					{0_m, 0_m, 0_deg}
				)
			)
		),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High)
	).ToPtr();
}