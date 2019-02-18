#include "Robot.h"

void Robot::RobotInit()
{
    cs::UsbCamera msLifeCam1 = CameraServer::GetInstance()->StartAutomaticCapture();
    msLifeCam1.SetResolution(320, 240);
    msLifeCam1.SetFPS(10);
    msLifeCam1.SetExposureAuto();
    pointer2msLifeCam1 = &msLifeCam1;
    Wait(2);

    dash->init();
    dash->PutString("Init Status", "Waiting");

    for(int x = 0; x <= 2; x++) {
        ypr[x] = 0.0;
    }

    lightRelay = new Relay(0, frc::Relay::Direction::kForwardOnly);
    stilts = new Stilts();
    arm = new Arm(dash);
    lift = new Lift(dash);
    intake = new Intake();
    tankdrive = new Tankdrive(1, 0, 1,2,3,4, myGyro, dash);
    myGyro = new OECPigeonIMU(tankdrive->GetTalonSRX());

    stickLeft = new OECJoystick(0);
    stickRight = new OECJoystick(1);
    stickUtil = new OECJoystick(2);

    dash->PutString("Version:", "0.7");
    dash->PutString("Init Status", "Complete");
}

void Robot::AutonomousInit() {
    pointer2msLifeCam1->SetExposureManual(12);
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    lightRelay->Set(frc::Relay::Value::kOn);
    myGyro->ResetAngle();
  //  tankdrive->ResetEncoders();
  //  pointer2msLifeCam1->SetExposureAuto();
}

double driveThrottle = 0.0;
double stiltsThrottle = 0.0;
double liftThrottle = 0.0;

void Robot::TeleopPeriodic() {
    driveThrottle = stickLeft->GetZ() / -2.0 + 0.5;
    tankdrive->SetPower(-1.0 * driveThrottle * stickLeft->GetY(), -1.0 * driveThrottle * stickRight->GetY());
    dash->PutNumber("Drive Throttle", driveThrottle);
    dash->PutNumber("Left Encoder Distance", tankdrive->GetLeftEncoderDist());
    dash->PutNumber("Right Encoder Distance", tankdrive->GetRightEncoderDist());

    stiltsThrottle = stickRight -> GetZ() / -2.0 + 0.5;
    dash->PutNumber("Stilts Throttle", stiltsThrottle);
    if(stickLeft->GetButton(3) && !stickLeft -> GetButton(2))
        stilts->SetFrontPower(stiltsThrottle);
    else if(!stickLeft->GetButton(3) && stickLeft -> GetButton(2))
        stilts->SetFrontPower(-1.0 * stiltsThrottle);
    else
        stilts->SetFrontPower(0.0);
    if(stickRight->GetButton(3) && !stickRight -> GetButton(2))
        stilts->SetRearPower(stiltsThrottle);
    else if(!stickRight->GetButton(3) && stickRight -> GetButton(2))
        stilts->SetRearPower(-1.0 * stiltsThrottle);
    else
        stilts->SetRearPower(0.0);
    if(stickLeft->GetButton(4))
        stilts->SetDrivePower(0.5);
    else if(stickLeft->GetButton(5))
        stilts->SetDrivePower(-0.5);
    else
        stilts->SetDrivePower(0.0);

    liftThrottle = stickUtil->GetZ() / -2.0 + 0.5;
    dash->PutNumber("Lift Throttle", liftThrottle);
    if(stickUtil->GetButton(3) && !stickUtil ->GetButton(2))
        lift->SetPower(liftThrottle);
    else if(!stickUtil->GetButton(3) && stickUtil ->GetButton(2))
        lift->SetPower(-1.0 * liftThrottle);
    else
        lift -> SetPower(0.0);
    
    dash->PutNumber("Arm Power", stickUtil->GetY());
    arm->SetPower(stickUtil->GetY(), true);

    if(stickUtil->GetButton(4) && !stickUtil->GetButton(5))
        intake->SetPower(0.5);
    else if(!stickUtil->GetButton(4) && stickUtil->GetButton(5))
        intake->SetPower(-0.5);
    else
        intake->SetPower(0.0);
}

void Robot::TestInit() {

}
void Robot::TestPeriodic() {
    
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
