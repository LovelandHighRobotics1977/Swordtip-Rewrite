#include "commands/Autonomous.h"

frc2::CommandPtr AutoRoutine::fireCubeOnly(DriveSubsystem *drive, CubeArmSubsystem *arm) {
	return frc2::SequentialCommandGroup(
		drive->ZeroOdometry({0_m, 0_m, 0_deg}),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High)
	).ToPtr();
}

frc2::CommandPtr AutoRoutine::Red::MidPickupCube(DriveSubsystem *drive, CubeArmSubsystem *arm) {
    return frc2::SequentialCommandGroup(
		drive->ZeroOdometry({0_m, 0_m, 0_deg}),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High),
		frc2::ParallelRaceGroup(
			frc2::SequentialCommandGroup(
				frc2::WaitUntilCommand( [drive] { return (drive->GetPose().X() > 85_in); } ),
				ArmCommand::BeginCubePickup(arm)
			),
			SwerveCommand::FollowPath(drive, 
				{0_m, 0_m, 0_deg},
				{
					{16_in, -10_in},
					{20_in, -50_in}
				}, 
				{120_in, -65_in, 0_deg}
			)
		),
		frc2::ParallelCommandGroup(
			ArmCommand::EndCubePickup(arm),
			SwerveCommand::FollowPath(drive, 
				{120_in, -65_in, 0_deg},
				{
					{20_in, -50_in},
					{16_in, -10_in}
				}, 
				{0_m, 0_m, 0_deg}
			)
		),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High)
	).ToPtr();
}

frc2::CommandPtr AutoRoutine::Red::DriveForward(DriveSubsystem *drive, CubeArmSubsystem *arm) {
	return frc2::SequentialCommandGroup(
		drive->ZeroOdometry({48_ft, 14_ft, 180_deg}),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High),
		frc2::ParallelRaceGroup(
			frc2::SequentialCommandGroup(
				frc2::WaitUntilCommand( [drive] { return (drive->GetPose().X() < 41_ft); } ),
				ArmCommand::BeginCubePickup(arm)
			),
			SwerveCommand::FollowPath(drive, 
				{48_ft, 14_ft, 180_deg},
				{
					{47_ft, 14_ft},
					{44_ft, 15_ft}
				}, 
				{35_ft, 16_ft, 180_deg}
			)
		),
		frc2::ParallelCommandGroup(
			ArmCommand::EndCubePickup(arm),
			SwerveCommand::FollowPath(drive, 
				{35_ft, 16_ft, 180_deg},
				{
					{44_ft, 16_ft},
					{47_ft, 14_ft}
				}, 
				{48_ft, 14_ft, 180_deg}
			)
		),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High)
	).ToPtr();
}

frc2::CommandPtr AutoRoutine::Odometry::TestOne(DriveSubsystem *drive) {
	return frc2::SequentialCommandGroup(
		drive->ZeroOdometry({48_ft, 14_ft, 0_deg}),
		SwerveCommand::FollowPath(drive, 
			{48_ft, 14_ft, 0_deg},
			{
				{48_ft, 16_ft},
				{46_ft, 16_ft}
			}, 
			{37_ft, 16_ft, 0_deg}
		)
	).ToPtr();
}