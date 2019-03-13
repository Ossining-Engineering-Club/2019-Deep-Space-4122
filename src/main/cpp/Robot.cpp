#include "Robot.h"
cs::UsbCamera msLifeCam1;
cs::UsbCamera msLifeCam2;
cs::VideoSink server;
void Robot::RobotInit()
{
    msLifeCam1 = CameraServer::GetInstance()->StartAutomaticCapture(0);
    msLifeCam2 = CameraServer::GetInstance()->StartAutomaticCapture(1);
    msLifeCam1.SetResolution(320, 240);
    msLifeCam1.SetFPS(15);
    msLifeCam1.SetExposureAuto();
    msLifeCam2.SetResolution(320, 240);
    msLifeCam2.SetFPS(15);
    msLifeCam2.SetExposureAuto();
    server = CameraServer::GetInstance()->GetServer();
    pointer2msLifeCam1 = &msLifeCam1;
    Wait(2);

    dash->init();
    dash->PutString("Init Status", "Waiting");

    for(int x = 0; x <= 2; x++) {
        ypr[x] = 0.0;
    }

    lightRelay = new Relay(0, frc::Relay::Direction::kForwardOnly);
    tankdrive = new Tankdrive(1, 0, 1,2,3,4, dash);

    stickLeft = new OECJoystick(0);
    stickRight = new OECJoystick(1);
    stickUtil = new OECJoystick(2);

    dash->PutString("Version:", "0.9");
    dash->PutString("Init Status", "Complete");
}

void Robot::AutonomousInit() {
    tankdrive->DriveStraightGyro(0.6, 80.0, 0.25, false);
    tankdrive->DriveStraightGyro(0.3, 15.0, 0.0, false);
    tankdrive->DriveCurveEncoder(-40, 50, 0.3, 0.0, false);
    tankdrive->DriveCurveEncoder(40, 43, 0.3, 0.0, true);

}
void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
    lightRelay->Set(frc::Relay::Value::kOn);
  //  tankdrive->ResetEncoders();
  //  pointer2msLifeCam1->SetExposureAuto();
}

double driveThrottle = 0.0;
double stiltsThrottle = 0.0;
double liftThrottle = 0.0;
bool lastBtn6 = false;
bool lastBtn11 = false;
int source = 0;
void Robot::TeleopPeriodic() {
    for(int x = 0; x <= 100; x++){
    if(stickRight->GetButton(6) && !lastBtn6){
        if(source == 0){
            server.SetSource(msLifeCam1);
            source = 1;
        }
        else if(source == 1){
            server.SetSource(msLifeCam2);
            source = 0;
        }
    }
    if(lastBtn11 && !stickUtil->GetButton(11)){
        //arm = new Arm(dash);
    }
    lastBtn11 = stickUtil->GetButton(11);
    

    driveThrottle = stickLeft->GetZ() / -2.0 + 0.5;
    tankdrive->SetPower(-1.0 * driveThrottle * stickLeft->GetY(), -1.0 * driveThrottle * stickRight->GetY());

    stiltsThrottle = stickRight -> GetZ() / -2.0 + 0.5;
    if(stickLeft->GetButton(3) && !stickLeft -> GetButton(2)){}
        //stilts->SetFrontPower(stiltsThrottle);
    else if(!stickLeft->GetButton(3) && stickLeft -> GetButton(2)){}
        //stilts->SetFrontPower(-1.0 * stiltsThrottle);
    else{}
        //stilts->SetFrontPower(0.0);
    if(stickRight->GetButton(3) && !stickRight -> GetButton(2)){}
        //stilts->SetRearPower(stiltsThrottle);
    else if(!stickRight->GetButton(3) && stickRight -> GetButton(2)){}
        //stilts->SetRearPower(-1.0 * stiltsThrottle);
    else{}
        //stilts->SetRearPower(0.0);
    if(stickLeft->GetButton(4)){}
        //stilts->SetDrivePower(0.5);
    else if(stickLeft->GetButton(5)){}
        //stilts->SetDrivePower(-0.5);
    else{}
        //stilts->SetDrivePower(0.0);

    liftThrottle = stickUtil->GetZ() / -2.0 + 0.5;
    
    if(stickUtil->GetButton(3) && !stickUtil ->GetButton(2)){}
        //lift->SetPower(liftThrottle);
    else if(!stickUtil->GetButton(3) && stickUtil ->GetButton(2)){}
        //lift->SetPower(-1.0 * liftThrottle);
    else
        //lift -> SetPower(0.0);

    //arm->SetPower(stickUtil->GetY(), stickUtil->GetButton(10));

    if(stickUtil->GetButton(4) && !stickUtil->GetButton(5)){}
        //intake->SetPower(0.35);
    else if(!stickUtil->GetButton(4) && stickUtil->GetButton(5)){}
        //intake->SetPower(-0.35);
    else{}
        //intake->SetPower(0.0);

        Wait(0.005);
    }
    dash->PutNumber("Arm Power", stickUtil->GetY());
    dash->PutNumber("Drive Throttle", driveThrottle);
    dash->PutNumber("Lift Throttle", liftThrottle);
    dash->PutNumber("Stilts Throttle", stiltsThrottle);
    //dash->PutNumber("Lift Encoder Position", lift->GetEncoderPosition());
}


void Robot::TestInit() {

}
void Robot::TestPeriodic() {
    
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
