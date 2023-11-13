// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef _ROBOT_H
#define _ROBOT_H

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;

  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;

  RobotContainer m_container;
};

#endif  // _ROBOT_H