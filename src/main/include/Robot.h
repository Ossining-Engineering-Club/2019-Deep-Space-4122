/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPIlib.h>
#include <ctre/Phoenix.h>
#include <wpi/ArrayRef.h>
#include "Lift.h"
#include "OECPigeonIMU.h"
#include "Tankdrive.h"
#include "Stilts.h"

using namespace frc;

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
private:
  double ypr[3];
  std::vector<double> *area;
  Tankdrive *tankdrive;
  OECPigeonIMU *myGyro;
  Stilts *stilts;
  Lift *lift;
  Relay *lightRelay;
  Joystick *stick2;
  Joystick *stick1;
  JoystickButton *stick1button6;
  SmartDashboard *dash;
};
