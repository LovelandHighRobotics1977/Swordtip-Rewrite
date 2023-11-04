#include "commands/ArmCommands.h"

frc2::FunctionalCommand ArmCommand::RaiseArm(CubeArmSubsystem *arm) {
    return frc2::FunctionalCommand(
		[arm] { arm->setAngle(false, false); },
		[arm] { arm->setAngle(true, false); },
		[arm] (bool interrupted) { arm->setAngle(false, false); },
		[arm] { return arm->getUpperSwitch(); }
	);
}

frc2::FunctionalCommand ArmCommand::LowerArm(CubeArmSubsystem *arm) {
	return frc2::FunctionalCommand(
		[arm] { arm->setAngle(false, false); },
		[arm] { arm->setAngle(false, true); },
		[arm] (bool interrupted) { arm->setAngle(false, false); },
		[arm] { return arm->getLowerSwitch(); }
	);
}

frc2::SequentialCommandGroup ArmCommand::FireCube(CubeArmSubsystem *arm, int target) {
	return frc2::SequentialCommandGroup(
		ArmCommand::RaiseArm(arm),
	    frc2::InstantCommand( [arm, target] { arm->setTarget(target); } ),
	    frc2::InstantCommand( [arm] { arm->setIntake(true); } ),
	    frc2::WaitCommand(0.5_s),
	    frc2::InstantCommand( [arm] { arm->setIntake(false); } )
	);
}

frc2::InstantCommand ArmCommand::EnableIntake(CubeArmSubsystem *arm) {
	return frc2::InstantCommand( [arm] { arm->setIntake(true); } );
}

frc2::InstantCommand ArmCommand::DisableIntake(CubeArmSubsystem *arm) {
	return frc2::InstantCommand( [arm] { arm->setIntake(false); } );
}

frc2::SequentialCommandGroup ArmCommand::BeginCubePickup(CubeArmSubsystem *arm) {
	return frc2::SequentialCommandGroup(
		ArmCommand::LowerArm(arm),
	    frc2::InstantCommand( [arm] { arm->setTarget(Mechanism::Intake::Target::Pickup); } ),
	    ArmCommand::EnableIntake(arm)
	);
}

frc2::InstantCommand ArmCommand::EndCubePickup(CubeArmSubsystem *arm) {
	return frc2::InstantCommand( 
		ArmCommand::DisableIntake(arm) 
	);
}