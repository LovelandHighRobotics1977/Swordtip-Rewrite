#include "commands/Autonomous.h"

frc2::CommandPtr AutoRoutine::fireCubeOnly(DriveSubsystem *drive, CubeArmSubsystem *arm) {
	return frc2::SequentialCommandGroup(
		drive->ZeroOdometry({0_m, 0_m, 0_deg}),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High)
	).ToPtr();
}

frc2::CommandPtr AutoRoutine::Red::DriveForward(DriveSubsystem *drive, CubeArmSubsystem *arm) {
	return frc2::SequentialCommandGroup(
		drive->ZeroOdometry({48_ft, 14_ft, 180_deg}),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High),
		frc2::ParallelRaceGroup(
			frc2::SequentialCommandGroup(
				frc2::WaitUntilCommand( [drive] { return (drive->GetPose().X() < 47_ft); } ),
				ArmCommand::BeginCubePickup(arm)
			),
			SwerveCommand::FollowPath(drive, 
				{48_ft, 14_ft, 180_deg},
				{
					{47_ft, 14_ft},
					{44_ft, 15_ft}
				}, 
				{33_ft, 16_ft, 180_deg}
			)
		),
		frc2::ParallelCommandGroup(
			ArmCommand::EndCubePickup(arm),
			SwerveCommand::FollowPath(drive, 
				{33_ft, 16_ft, 180_deg},
				{
					{44_ft, 16_ft},
					{47_ft, 14_ft}
				}, 
				{48_ft, 14_ft, 210_deg}
			)
		),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High)
	).ToPtr();
}

frc2::CommandPtr AutoRoutine::Red::MidBalance(DriveSubsystem *drive, CubeArmSubsystem *arm) {
	return frc2::SequentialCommandGroup(
		drive->ZeroOdometry({48_ft, 8.4_ft, 180_deg}),
		ArmCommand::FireCube(arm, Mechanism::Intake::Target::High),
		SwerveCommand::FollowPath(drive,
			{48_ft, 8.4_ft, 180_deg},
			{
				{36_ft , 8.4_ft}
			},
			{42_ft , 8.4_ft, 180_deg}
		)
	).ToPtr();
}