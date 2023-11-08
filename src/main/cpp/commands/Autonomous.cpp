#include "commands/Autonomous.h"

frc2::CommandPtr AutoRoutine::autoOne(DriveSubsystem *drive, CubeArmSubsystem *arm) {
    return frc2::SequentialCommandGroup(
		drive->ZeroOdometry({0_m, 0_m, 0_deg}),
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
		SwerveCommand::FollowPath(drive,
			{1.4_m, -1.9_m, 0_deg},
			{},
			{2.4_m, -1.9_m, 0_deg}
		),
		frc2::WaitCommand(0.5_s),
		frc2::ParallelCommandGroup(
			ArmCommand::EndCubePickup(arm),
			SwerveCommand::FollowPath(drive,
					{2.4_m, -1.9_m, 0_deg},
					{
						{0.3_m, -1.9_m}
					},
					{0_m, 0_m, 0_deg}
				)
		),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High)
	).ToPtr();
}