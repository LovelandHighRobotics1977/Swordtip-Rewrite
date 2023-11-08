#include "commands/ArmCommands.h"

frc2::SequentialCommandGroup ArmCommand::FireCube(CubeArmSubsystem *arm, int target) {
	return frc2::SequentialCommandGroup(
		arm->RaiseArm(),
		frc2::InstantCommand( [arm, target] { arm->setTarget(target); } , {arm} ),
		frc2::ParallelRaceGroup(
			arm->ShootCube(),
	    	frc2::WaitCommand(0.5_s)
		)
	);
}

frc2::SequentialCommandGroup ArmCommand::BeginCubePickup(CubeArmSubsystem *arm) {
	return frc2::SequentialCommandGroup(
		arm->LowerArm(),
	    arm->PickupCube()
	);
}

frc2::SequentialCommandGroup ArmCommand::EndCubePickup(CubeArmSubsystem *arm) {
	return frc2::SequentialCommandGroup( 
		arm->StopIntake(),
		arm->RaiseArm()
	);
}