#include "Robot.h"

void Robot::RobotInit()
{
    cs::UsbCamera msLifeCam1 = CameraServer::GetInstance()->StartAutomaticCapture();
    msLifeCam1.SetResolution(640, 480);
    msLifeCam1.SetFPS(10);
    msLifeCam1.SetExposureManual(12);
    Wait(2);
    dash->init();
    dash->PutString("Init Status", "Waiting");
    for(int x = 0; x <= 2; x++) {
        ypr[x] = 0.0;
    }
    //mynttable = nt::NetworkTableInstance::GetTable("GRIP/myContoursReport");
    lightRelay = new Relay(0, frc::Relay::Direction::kForwardOnly);
    stilts = new Stilts();
    stick1 = new Joystick(1);
    stick2 = new Joystick (2);
    tankdrive = new Tankdrive(1, 0, 1,2,3,4, myGyro, dash);
    myGyro = new OECPigeonIMU(tankdrive->GetTalonSRX());
    lift = new Lift();
    //leftDrive = new Talon(1);
    //rightDrive = new Talon(0);
    stick1button6 = new JoystickButton(stick1, 6);
    dash->PutString("Version:", "0.7");
    dash->PutString("Init Status", "Complete");
}

void Robot::AutonomousInit() {
    tankdrive->DriveCurveEncoder(-200, 90, 0.2);
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    lightRelay->Set(frc::Relay::Value::kOn);
    myGyro->ResetAngle();
    tankdrive->ResetEncoders();
    


}
void Robot::TeleopPeriodic() {
    if(false){
        //lift->SetPower(stick1->GetY());
    }
    else{
        lift->SetPower(0.0);
    //tankdrive->SetPower(stick1->GetY(), stick2->GetY());
    stilts->SetFrontPower(stick1->GetY());
    stilts->SetRearPower(stick2->GetY());
}
    dash->PutNumber("Gyro Angle", myGyro->GetYaw(OECPigeonIMU::AngleUnits::degrees));
    dash->PutNumber("Encoder Distance", (tankdrive->GetEncoderDist(Tankdrive::DriveSide::left)+tankdrive->GetEncoderDist(Tankdrive::DriveSide::right))/2);
}

void Robot::TestInit() {

}
void Robot::TestPeriodic() {
    tankdrive->SetPower(0.4*stick1->GetY(), 0.4*stick2->GetY());
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
