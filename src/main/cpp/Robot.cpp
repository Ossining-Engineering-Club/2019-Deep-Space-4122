#include "Robot.h"
cs::UsbCamera msLifeCam1;
cs::UsbCamera msLifeCam2;
cs::VideoSink server;
bool reverseDrive = false;
void Robot::RobotInit()
{
    msLifeCam1 = CameraServer::GetInstance()->StartAutomaticCapture(0);
    msLifeCam2 = CameraServer::GetInstance()->StartAutomaticCapture(1);
    msLifeCam1.SetResolution(160, 120);
    msLifeCam1.SetFPS(15);
    msLifeCam1.SetExposureAuto();
    msLifeCam2.SetResolution(160, 120);
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
    stilts = new Stilts();
    intake = new Intake();
    lift = new Lift(dash);
    arm = new Arm(dash);

    myVision = new Vision();

    tankdrive->stickLeft = new OECJoystick(0);
    tankdrive->stickRight = new OECJoystick(1);
    tankdrive->stickUtil = new OECJoystick(2);

    dash->PutString("Version:", VERSION_NUMBER);
    dash->PutString("Init Status", "Complete");
}

void Robot::AutonomousInit() {
    //DigitalInput CenterSideDIO(8);
    DigitalInput LeftRightDIO(0);

    /*if(!CenterSideDIO.Get()){
        double RightLeft = 1.0;
        if(LeftRightDIO.Get()){RightLeft = -1.0;
            dash->PutString("Autonomous Path", "Center Right");
        }
        else{
            dash->PutString("Autonomous Path", "Center Left");
        }
        tankdrive->DriveStraightGyro(STRAIGHT_SPEED_1, CENTER_STRAIGHT_DIST_1, .25, false);
        tankdrive->TurnToHeading(TURN_SPEED_1, -1.0 * RightLeft * CENTER_TURN_HEADING_1, 1.5);
        //drop arm
        tankdrive->DriveStraightGyro(STRAIGHT_SPEED_1, CENTER_STRAIGHT_DIST_2, .25, false);
        tankdrive->TurnToHeading(TURN_SPEED_1, RightLeft * CENTER_TURN_HEADING_1, 1.5);
    }*/
    //else{
        double RightLeft = 1.0; // Insert jumper for right side path
        if(LeftRightDIO.Get()){
            RightLeft = -1.0;     //Run left side path
            dash->PutString("Autonomous Path", "Side Right");
        }
        else{
            dash->PutString("Autonomous Path", "Side Left");
        }

        tankdrive->DriveStraightGyro(STRAIGHT_SPEED_1, SIDE_STRAIGHT_DIST_1, .25, false);
        
        tankdrive->TurnToHeading(TURN_SPEED_1, -1.0 * RightLeft * SIDE_TURN_HEADING_1, 1.5);
        tankdrive->DriveStraightGyro(STRAIGHT_SPEED_2, SIDE_STRAIGHT_DIST_2, 0.0, true);
        tankdrive->TurnToHeading(TURN_SPEED_2, RightLeft * SIDE_TURN_HEADING_2, 3.0);
        //tankdrive->DriveVision(36.0, STRAIGHT_SPEED_3);
        //tankdrive->SetPower(0.0, 0.0);
        //drop arm
        //tankdrive->TurnToHeading(TURN_SPEED_3, RightLeft * FINAL_TURN_HEADING, 1.5);
    //}
}
double driveThrottle = 0.0;
double stiltsThrottle = 0.0;
double liftThrottle = 0.0;
bool lastBtn6 = false;
bool lastBtn11 = false;
bool lastLeftTrigger = false;
int source = 0;
int armMode = 0;

void Robot::AutonomousPeriodic() {
    lastLeftTrigger = tankdrive->stickLeft->GetButton(1);
    for(int x = 0; x <= 100; x++){
    
    if(tankdrive->stickRight->GetButton(6) && !lastBtn6){
        if(source == 0){
            server.SetSource(msLifeCam1);
            source = 1;
        }
        else if(source == 1){
            server.SetSource(msLifeCam2);
            source = 0;
        }
    }
    if(lastBtn11 && !tankdrive->stickUtil->GetButton(11)){
        arm = new Arm(dash);
    }
    lastBtn11 = tankdrive->stickUtil->GetButton(11);
    

    driveThrottle = tankdrive->stickLeft->GetZ() / -2.0 + 0.5;
    if(tankdrive->stickLeft->GetButton(1))
        driveThrottle = 0.45;
    if(!reverseDrive){
        tankdrive->SetPower(-1.0 * driveThrottle * tankdrive->stickLeft->GetY(), -1.0 * driveThrottle * tankdrive->stickRight->GetY());
    }
    else{
        //tankdrive->SetPower(driveThrottle * tankdrive->stickRight->GetY(), driveThrottle * tankdrive->stickLeft->GetY());
        tankdrive->SetPower(-1.0 * driveThrottle * tankdrive->stickLeft->GetY(), -1.0 * driveThrottle * tankdrive->stickRight->GetY());
    }

    stiltsThrottle = tankdrive->stickRight -> GetZ() / -2.0 + 0.5;
    if(tankdrive->stickLeft->GetButton(3) && !tankdrive->stickLeft -> GetButton(2)){
        stilts->SetFrontPower(stiltsThrottle);
    }
    else if(!tankdrive->stickLeft->GetButton(3) && tankdrive->stickLeft -> GetButton(2)){
        stilts->SetFrontPower(-1.0 * stiltsThrottle);
    }
    else{
        stilts->SetFrontPower(0.0);
    }
    if(tankdrive->stickRight->GetButton(3) && !tankdrive->stickRight -> GetButton(2)){
        stilts->SetRearPower(stiltsThrottle);
    }
    else if(!tankdrive->stickRight->GetButton(3) && tankdrive->stickRight -> GetButton(2)){
        stilts->SetRearPower(-1.0 * stiltsThrottle);
    }
    else{
        stilts->SetRearPower(0.0);
    }
    if(tankdrive->stickLeft->GetButton(4)){
        stilts->SetDrivePower(0.5);
    }
    else if(tankdrive->stickLeft->GetButton(5)){
        stilts->SetDrivePower(-0.5);
    }
    else{
        stilts->SetDrivePower(0.0);
    }

    liftThrottle = tankdrive->stickUtil->GetZ() / -2.0 + 0.5;
    
        if(abs(tankdrive->stickUtil->GetY()) > ARM_MANUAL_THRES || tankdrive->stickUtil->GetButton(2) || tankdrive->stickUtil->GetButton(3))
       { armMode = 0;
       dash->PutString("arm mode", "Manual");
       }
    else if(tankdrive->stickUtil->GetButton(1))
    {
        armMode = 1;
        dash->PutString("arm mode", "Load");
}
    else if(tankdrive->stickUtil->GetButton(7))
        armMode = 2;
    else if(tankdrive->stickUtil->GetButton(6))
        armMode = 3;
    else if(tankdrive->stickUtil->GetButton(8))
        armMode = 4;
    if(lastLeftTrigger && !tankdrive->stickLeft->GetButton(1))
        reverseDrive = !reverseDrive;
        
    if(armMode == 0){
        
        arm->SetPower(tankdrive->stickUtil->GetY(), tankdrive->stickUtil->GetButton(10));

        if(tankdrive->stickUtil->GetButton(3)){
            lift->SetPower(liftThrottle);
        }
        else if(tankdrive->stickUtil->GetButton(2)){
            lift->SetPower(-1.0 * liftThrottle);
        }
        else{
            lift -> SetPower(0.0);
        }
    }

    else if(armMode == 1){
        dash->PutString("arm state", "Load");
        arm->SetToPosition(0.4, ARM_PICKUP_POS);
        lift->SetToPosition(0.3, LIFT_PICKUP_POS);
    }
    else if(armMode == 2){
        arm->SetToPosition(0.4, ARM_HOLD_POS);
        lift->SetToPosition(0.3, LIFT_HOLD_POS);
    }
    else if(armMode == 3){
        arm->SetToPosition(0.4, ARM_LEV_2_POS);
        lift->SetToPosition(0.55, LIFT_LEV_2_POS);
    }
    else if(armMode == 4){
        arm->SetToPosition(0.4, ARM_LEV_3_POS);
        lift->SetToPosition(0.55, LIFT_LEV_3_POS);
    }

    if(tankdrive->stickUtil->GetButton(4) && !tankdrive->stickUtil->GetButton(5)){
        intake->SetPower(0.6);
    }
    else if(!tankdrive->stickUtil->GetButton(4) && tankdrive->stickUtil->GetButton(5)){
        intake->SetPower(-0.6);
    }
    else{
        intake->SetPower(0.0);
    }

        Wait(0.005);
    }
    dash->PutNumber("Arm Power", tankdrive->stickUtil->GetY());
    dash->PutNumber("Drive Throttle", driveThrottle);
    dash->PutNumber("Lift Throttle", liftThrottle);
    dash->PutNumber("Stilts Throttle", stiltsThrottle);
    dash->PutNumber("Lift Encoder Position", lift->GetEncoderPosition());
}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
    //for(int x = 0; x <= 100; x++){
    if(tankdrive->stickUtil->GetButton(7))
        arm->SetToPosition(0.15, -50.0);
    if(lastLeftTrigger && !tankdrive->stickLeft->GetButton(1))
        reverseDrive = !reverseDrive;
    lastLeftTrigger = tankdrive->stickLeft->GetButton(1);
    if(tankdrive->stickRight->GetButton(6) && !lastBtn6){
        if(source == 0){
            server.SetSource(msLifeCam1);
            source = 1;
        }
        else if(source == 1){
            server.SetSource(msLifeCam2);
            source = 0;
        }
    }
    if(lastBtn11 && !tankdrive->stickUtil->GetButton(11)){
        arm = new Arm(dash);
    }
    lastBtn11 = tankdrive->stickUtil->GetButton(11);
    

    driveThrottle = tankdrive->stickLeft->GetZ() / -2.0 + 0.5;
    if(tankdrive->stickLeft->GetButton(1))
        driveThrottle = 0.45;
    if(!reverseDrive){
        tankdrive->SetPower(-1.0 * driveThrottle * tankdrive->stickLeft->GetY(), -1.0 * driveThrottle * tankdrive->stickRight->GetY());
    }
    else{
        tankdrive->SetPower(-1.0 * driveThrottle * tankdrive->stickLeft->GetY(), -1.0 * driveThrottle * tankdrive->stickRight->GetY());
        //tankdrive->SetPower(driveThrottle * tankdrive->stickRight->GetY(), driveThrottle * tankdrive->stickLeft->GetY());
    }

    stiltsThrottle = tankdrive->stickRight->GetZ() / -2.0 + 0.5;
    
    if(tankdrive->stickLeft->GetButton(3) && !tankdrive->stickLeft -> GetButton(2)){
        stilts->SetFrontPower(stiltsThrottle);
    }
    else if(!tankdrive->stickLeft->GetButton(3) && tankdrive->stickLeft -> GetButton(2)){
        stilts->SetFrontPower(-1.0 * stiltsThrottle);
    }
    else{
        stilts->SetFrontPower(0.0);
    }

    if(tankdrive->stickRight->GetButton(3) && !tankdrive->stickRight -> GetButton(2)){
        stilts->SetRearPower(stiltsThrottle);
    }
    else if(!tankdrive->stickRight->GetButton(3) && tankdrive->stickRight -> GetButton(2)){
        stilts->SetRearPower(-1.0 * stiltsThrottle);
    }
    else{
        stilts->SetRearPower(0.0);
    }
    
    if(tankdrive->stickLeft->GetButton(4)){
        stilts->SetDrivePower(0.5);
    }
    else if(tankdrive->stickLeft->GetButton(5)){
        stilts->SetDrivePower(-0.5);
    }
    else{

        stilts->SetDrivePower(0.0);
    }

    liftThrottle = tankdrive->stickUtil->GetZ() / -2.0 + 0.5;
    
    if(abs(tankdrive->stickUtil->GetY()) > ARM_MANUAL_THRES || tankdrive->stickUtil->GetButton(2) || tankdrive->stickUtil->GetButton(3))
       { armMode = 0;
       dash->PutString("arm mode", "Manual");
       }
    else if(tankdrive->stickUtil->GetButton(1))
    {
        armMode = 1;
        dash->PutString("arm mode", "Load");
}
    else if(tankdrive->stickUtil->GetButton(7))
        armMode = 2;
    else if(tankdrive->stickUtil->GetButton(6))
        armMode = 3;
    else if(tankdrive->stickUtil->GetButton(8))
        armMode = 4;
    if(lastLeftTrigger && !tankdrive->stickLeft->GetButton(1))
        reverseDrive = !reverseDrive;
        
    if(armMode == 0){
        
        arm->SetPower(tankdrive->stickUtil->GetY(), tankdrive->stickUtil->GetButton(10));

        if(tankdrive->stickUtil->GetButton(3)){
            lift->SetPower(liftThrottle);
        }
        else if(tankdrive->stickUtil->GetButton(2)){
            lift->SetPower(-1.0 * liftThrottle);
        }
        else{
            lift -> SetPower(0.0);
        }
    }

    else if(armMode == 1){
        dash->PutString("arm state", "Load");
        arm->SetToPosition(0.4, ARM_PICKUP_POS);
        lift->SetToPosition(0.3, LIFT_PICKUP_POS);
    }
    else if(armMode == 2){
        arm->SetToPosition(0.4, ARM_HOLD_POS);
        lift->SetToPosition(0.3, LIFT_HOLD_POS);
    }
    else if(armMode == 3){
        arm->SetToPosition(0.4, ARM_LEV_2_POS);
        lift->SetToPosition(0.55, LIFT_LEV_2_POS);
    }
    else if(armMode == 4){
        arm->SetToPosition(0.4, ARM_LEV_3_POS);
        lift->SetToPosition(0.55, LIFT_LEV_3_POS);
    }

    if(tankdrive->stickUtil->GetButton(4) && !tankdrive->stickUtil->GetButton(5)){
        intake->SetPower(0.6);
    }
    else if(!tankdrive->stickUtil->GetButton(4) && tankdrive->stickUtil->GetButton(5)){
        intake->SetPower(-0.6);
    }
    else{
        intake->SetPower(0.0);
    }
        Wait(0.005);
    //}
    dash->PutNumber("Arm Power", tankdrive->stickUtil->GetY());
    dash->PutNumber("Drive Throttle", driveThrottle);
    dash->PutNumber("Lift Throttle", liftThrottle);
    dash->PutNumber("Stilts Throttle", stiltsThrottle);
    dash->PutNumber("Lift Encoder Position", lift->GetEncoderPosition());
}


void Robot::TestInit() {

}
void Robot::TestPeriodic() {
    
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
