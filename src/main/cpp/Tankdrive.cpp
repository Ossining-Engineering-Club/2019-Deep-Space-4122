#include "Tankdrive.h"
#include <iostream>

using namespace std;
Tankdrive::Tankdrive(int leftPort, int rightPort, int leftEncoder1, int leftEncoder2, int rightEncoder1, int rightEncoder2, SmartDashboard *dash):
    vision()
{
    this->dash = dash;
    pidController = new OECPIDController(CURVED_KP, CURVED_KI, CURVED_KD, CURVED_CORRECTION);
    IMUTalonSRX = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(2);
    LeftFrontDrive = new rev::CANSparkMax(25, rev::CANSparkMax::MotorType::kBrushless);
    LeftBackDrive = new rev::CANSparkMax(26, rev::CANSparkMax::MotorType::kBrushless);
    RightFrontDrive = new rev::CANSparkMax(27, rev::CANSparkMax::MotorType::kBrushless);
    RightBackDrive = new rev::CANSparkMax(28, rev::CANSparkMax::MotorType::kBrushless);
    pigeonIMU = new OECPigeonIMU(01); //CHANGE THIS BACK!!!
    pigeonIMU->ResetAngle();
    rightNeoEncoder = new rev::CANEncoder(27);
    rightNeoEncoder = new rev::CANEncoder(25);
    leftEncoder = new frc::Encoder(leftEncoder1, leftEncoder2, false, CounterBase::EncodingType::k4X);
    rightEncoder = new frc::Encoder(rightEncoder1, rightEncoder2, true, CounterBase::EncodingType::k4X);
    rightEncoder->SetDistancePerPulse(RIGHT_ENCODER_CONST);
    leftEncoder->SetDistancePerPulse(LEFT_ENCODER_CONST);
    leftEncoder->Reset();
    rightEncoder->Reset();
    throttle = 1.0;
    myTimer = new Timer();
    loopMode = internal;
}
void Tankdrive::SetPower(double leftPower, double rightPower){
    LeftBackDrive->Set(leftPower * throttle * STRAIGHT_DRIVE_CORRECTION);
    RightBackDrive->Set(-1.0 * rightPower * throttle * (2-STRAIGHT_DRIVE_CORRECTION));
    LeftFrontDrive->Set(leftPower * throttle * STRAIGHT_DRIVE_CORRECTION);
    RightFrontDrive->Set(-1.0 * rightPower * throttle * (2- STRAIGHT_DRIVE_CORRECTION));
}
void Tankdrive::SetThrottle(double throttle){
    Tankdrive::throttle = throttle;
}
double Tankdrive::GetLeftEncoderDist(){
    return leftEncoder->GetDistance();
}

int Tankdrive::GetLeftEncoderRaw(){return leftEncoder->GetRaw();}
double Tankdrive::GetRightEncoderDist(){
    return rightEncoder->GetDistance();
}
void Tankdrive::AlignRobotVision(double currentDist, double targetAngle){
    dash->PutNumber("Current Heading", pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees));
    TurnToTarget(0.05);
    vision.Update();
    double dist = vision.GetDistance(0);
    double angle = fmod(fmod(pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees), 360.0)+360.0, 360.0); - targetAngle;
    double targetHeading = targetAngle + angle * ((dist+36.0)/24.0);
    double driveDist = dist-24.0;
    double power;
    if(targetHeading > fmod(fmod(pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees), 360.0)+360.0, 360.0)){
        power = -0.05;
    }
    else
        power = 0.05;
    while(abs(targetHeading-fmod(fmod(pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees), 360.0)+360.0, 360.0)) < 2.0){
        SetPower(power, -1.0*power);
    }
    if(targetAngle > fmod(fmod(pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees), 360.0)+360.0, 360.0)){
        power = -0.05;
    }
    else
        power = 0.05;
    while(abs(targetAngle-fmod(fmod(pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees), 360.0)+360.0, 360.0)) < 2.0){
        SetPower(power, -1.0*power);
    }
    DriveStraightGyro(0.1, driveDist, 0.0, true);
}



void Tankdrive::TurnToTarget(double power){
    vision.Update();
    double dist = vision.GetDistance(0);
    dash->PutNumber("Distance", dist);
    double xPos = (vision.GetX(0) + vision.GetX(1)) / 2;
    dash->PutNumber("X Position", xPos);
    while(xPos <= 160 - PIXEL_ACCURACY || xPos >= 160 + PIXEL_ACCURACY){
        SetPower(-1.0*power * (160-xPos) * (0.02 * dist) * (0.02), power *(160-xPos) * (0.02) * (0.02 * dist));
        vision.Update();
        xPos = (vision.GetX(0) + vision.GetX(1)) / 2;
    }
}
int Tankdrive::GetRightEncoderRaw(){return rightEncoder->GetRaw();}
void Tankdrive::TurnToHeading(double maxPower, double headingDegrees){
    dash->PutNumber("Current Heading", pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees));
    double turnError = 0.0;
    double correction = 0.0;
    pidController = new OECPIDController(TURN_KP, TURN_KI, TURN_KD, maxPower);
    while(abs(headingDegrees-pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees)) > TURN_ACCURACY){
        correction = pidController->GetPIDCorrection(pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees)-headingDegrees);
        SetPower(correction, -1.0*correction);
    }
    SetPower(0.0, 0.0);
}
OECPigeonIMU* Tankdrive::GetPigeonIMU(){
    return pigeonIMU;
}
void Tankdrive::DriveStraightGyro(double power, double distInches, double startupTime, bool stopAtEnd){
    pidController = new OECPIDController(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD, STRAIGHT_CORRECTION);
    ResetEncoders();
    double startHeading = pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees);
    double pow = 0.0;
    double gyroError = 0.0;
    double correction;
    myTimer->Stop();
    myTimer->Reset();
    myTimer->Start();
    while(startupTime > myTimer->Get() && abs(GetLeftEncoderDist() + GetRightEncoderDist()) < abs(2.0*distInches) - 2.0*STOP_DIST){
        gyroError = startHeading - pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees);
        pow = (myTimer->Get()/startupTime)*power;
        correction = pidController->GetPIDCorrection(gyroError);
        SetPower(pow + correction, pow - correction);
    }
    while(abs(GetLeftEncoderDist() + GetRightEncoderDist()) < abs(2.0*distInches) - 2.0*STOP_DIST){
        correction = pidController->GetPIDCorrection(gyroError);
        SetPower(power + correction, power - correction);
    }

    if(stopAtEnd){
        if(abs(power) < 0.2)
            pow = power;
        else 
            pow = (abs(power)/power)*STOP_POWER;
    }
    else{
        pow = power;
    }

    while(abs(GetLeftEncoderDist() + GetRightEncoderDist()) < abs(2.0*distInches)){
        correction = pidController->GetPIDCorrection(gyroError);
        SetPower(pow + correction, pow - correction);
    }
    if(stopAtEnd)
        SetPower(0.0, 0.0);
}
void Tankdrive::DriveCurveEncoder(double radius, double degrees, double avgPower, double startupTime, bool stopAtEnd){
    double tempPower = 0.0;
    pidController = new OECPIDController(CURVED_KP, CURVED_KI, CURVED_KD, CURVED_CORRECTION);
    double leftPower = avgPower * (1 - 0.5*DRIVEBASE_WIDTH/(radius*TURN_RADIUS_CORRECTION));
    double rightPower = avgPower * (1 + 0.5*DRIVEBASE_WIDTH/(radius*TURN_RADIUS_CORRECTION));
    double scaleValue = avgPower/(0.5*(leftPower + rightPower));
    //leftPower *= scaleValue;
    //rightPower *= scaleValue;
    if(leftPower < rightPower){
        //leftPower = leftPower * 1.0568 - 0.0615;
        //rightPower = rightPower * 0.9561 + 0.062;
    }
    else if(rightPower < leftPower){
        //rightPower = rightPower * 1.0568 - 0.0615;
        //leftPower = leftPower * 0.9561 + 0.062;
    }
    dash->PutNumber("Left Predicted", leftPower);
    dash->PutNumber("Right Predicted", rightPower);
    double totalDist = (degrees/360*2*PI*radius);
    double avgDist = 0;
    double targetRight;
    double targetLeft;
    double leftEnc;
    double rightEnc;
    double error;
    double correction;
    ResetEncoders();
    //SetPower(leftPower, rightPower);
    double leftTotal = 0.0;
    double rightTotal = 0.0;
    int count = 0;
    myTimer->Stop();
    myTimer->Reset();
    myTimer->Start();
    while(abs(avgDist) < abs(totalDist)){
        //dash->PutNumber("Timer time", myTimer->Get());
        if(myTimer->Get() < startupTime){
            tempPower = (myTimer->Get()/startupTime)*avgPower;
            //dash->PutNumber("power", tempPower);
            leftPower = tempPower * (1 - 0.5*DRIVEBASE_WIDTH/(radius*TURN_RADIUS_CORRECTION));
            rightPower = tempPower * (1 + 0.5*DRIVEBASE_WIDTH/(radius*TURN_RADIUS_CORRECTION));
            //dash->PutString("startup status", "in progress");
        }
        else{
            leftPower = avgPower * (1 - 0.5*DRIVEBASE_WIDTH/(radius*TURN_RADIUS_CORRECTION));
            rightPower = avgPower * (1 + 0.5*DRIVEBASE_WIDTH/(radius*TURN_RADIUS_CORRECTION));
            //dash->PutString("startup status", "complete");
        }
        leftEnc = GetLeftEncoderDist();
        rightEnc = GetRightEncoderDist();
        //dash->PutNumber("leftEncoder", GetLeftEncoderDist());
        //dash->PutNumber("rightEncoder", GetRightEncoderDist());
        avgDist = (leftEnc + rightEnc)/2.0;
        targetRight = avgDist + (avgDist*DRIVEBASE_WIDTH)/(2.0*radius);
        targetLeft = avgDist - (avgDist*DRIVEBASE_WIDTH)/(2.0*radius);
        error = (leftEnc - targetLeft) - (rightEnc-targetRight);
        //dash->PutNumber("Error", error);
        correction = pidController->GetPIDCorrection(error);
        if(abs(avgDist) > 0.5*abs(totalDist)){
            leftTotal += leftPower + correction;
            rightTotal += rightPower - correction;
            count ++;
        }
        SetPower(leftPower* (1 + correction), rightPower * (1 - correction));
    }

    dash->PutNumber("Correction", correction);
    dash->PutNumber("Left Average", leftTotal/count);
    dash->PutNumber("Right Average", rightTotal/count);
    if(stopAtEnd)
        SetPower(0.0,0.0);
}
ctre::phoenix::motorcontrol::can::WPI_TalonSRX* Tankdrive::GetTalonSRX(){
    return IMUTalonSRX;
}
void Tankdrive::DriveGyro(double degreesPerInch, double degrees, double avgPower, double timeoutSec){
    pidController->ResetIntegral();
    double radiansPerInch = (3.14159265358979)*degreesPerInch/180.0;
    double inchPerRadDiff = DRIVEBASE_WIDTH;
    double leftPower, rightPower;
    frc::Timer *myTimer;
    myTimer = new Timer();
    myTimer->Reset();
    myTimer->Start();
    leftPower = avgPower-avgPower*0.5*radiansPerInch*inchPerRadDiff;
    rightPower = avgPower+avgPower*0.5*radiansPerInch*inchPerRadDiff;
    dash->PutNumber("RightPower", rightPower);
    dash->PutNumber("LeftPower", leftPower);
    cout << "Right Power" << rightPower << endl;
    cout <<"Left Power " << leftPower << endl;
    pigeonIMU->ResetAngle();
    SetPower(leftPower, rightPower);
    while(abs(pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees)) < abs(degrees)){
        double avgDist = GetLeftEncoderDist()/2.0+GetRightEncoderDist()/2.0;
        double correction = pidController->GetPIDCorrection(pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees) - avgDist*degreesPerInch);
        SetPower(leftPower - correction, rightPower + correction);
    }
    SetPower(0.0, 0.0);
}
void Tankdrive::DriveGyroByRadius(double radius, double degrees, double avgPower, double timeoutSec){
    if(degrees < 0){
        double angleROC = 180.0/(radius*PI);
        cout << "Angle ROC" << angleROC << endl;
        //DriveGyro(-1*angleROC, degrees, avgPower, timeoutSec);
    }
    else{
        double angleROC = 360.0/(radius*PI);
        //DriveGyro(angleROC, degrees, avgPower, timeoutSec);
    }
}
void Tankdrive::ResetEncoders(){
    rightEncoder->Reset();
    leftEncoder->Reset();
}
