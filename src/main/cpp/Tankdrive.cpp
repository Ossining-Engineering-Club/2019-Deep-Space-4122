#include "Tankdrive.h"
#include <iostream>

using namespace std;
Tankdrive::Tankdrive(int leftPort, int rightPort, int leftEncoder1, int leftEncoder2, int rightEncoder1, int rightEncoder2, OECPigeonIMU *pigeonIMU, SmartDashboard *dash){
    this->dash = dash;
    this->pigeonIMU = pigeonIMU;
    pidController = new OECPIDController(CURVED_KP, CURVED_KI, CURVED_KD, CURVED_CORRECTION);
    LeftFrontDrive = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(01);
    LeftBackDrive = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(02);
    RightFrontDrive = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(03);
    RightBackDrive = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(04);
    leftEncoder = new frc::Encoder(leftEncoder1, leftEncoder2, false, CounterBase::EncodingType::k4X);
    rightEncoder = new frc::Encoder(rightEncoder1, rightEncoder2, true, CounterBase::EncodingType::k4X);
    rightEncoder->SetDistancePerPulse(ENCODER_CONST);
    leftEncoder->SetDistancePerPulse(ENCODER_CONST);
    leftEncoder->Reset();
    rightEncoder->Reset();
    throttle = 1.0;
    loopMode = internal;
}
void Tankdrive::SetPower(double leftPower, double rightPower){
    LeftBackDrive->Set(leftPower * throttle);
    RightBackDrive->Set(-1.0 * rightPower * throttle);
    LeftFrontDrive->Set(leftPower * throttle);
    RightFrontDrive->Set(-1.0 * rightPower * throttle);
}
void Tankdrive::SetThrottle(double throttle){
    Tankdrive::throttle = throttle;
}
double Tankdrive::GetEncoderDist(Tankdrive::DriveSide encoderSide){
    if(encoderSide = DriveSide::left)
        return leftEncoder->GetDistance();
    else
        return rightEncoder->GetDistance();
}
double Tankdrive::GetLeftEncoderDist(){
    return leftEncoder->GetDistance();
}
double Tankdrive::GetRightEncoderDist(){
    return rightEncoder->GetDistance();
}
void Tankdrive::DriveCurveEncoder(double radius, double degrees, double avgPower){
    pidController->ResetIntegral();
    double leftPower = avgPower * (1 - 0.5*DRIVEBASE_WIDTH/radius);
    double rightPower = avgPower * (1 + 0.5*DRIVEBASE_WIDTH/radius);
    leftPower = (abs(leftPower)/leftPower)*pow(abs(leftPower), TURN_GAMMA);
    rightPower = (abs(rightPower)/rightPower)*pow(abs(rightPower), TURN_GAMMA);
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
    SetPower(leftPower, rightPower);
    double leftTotal = 0.0;
    double rightTotal = 0.0;
    int count = 0;
    while(abs(avgDist) < abs(totalDist)){
        leftEnc = GetLeftEncoderDist();
        rightEnc = GetRightEncoderDist();
        dash->PutNumber("leftEncoder", GetLeftEncoderDist());
        dash->PutNumber("rightEncoder", GetRightEncoderDist());
        avgDist = (leftEnc + rightEnc)/2.0;
        targetRight = avgDist + (avgDist*DRIVEBASE_WIDTH)/(2.0*radius);
        targetLeft = avgDist - (avgDist*DRIVEBASE_WIDTH)/(2.0*radius);
        error = (leftEnc - targetLeft) - (rightEnc-targetRight);
        dash->PutNumber("Error", error);
        correction = pidController->GetPIDCorrection(error);
        dash->PutNumber("Left Actual", leftPower + correction);
        dash->PutNumber("Right Actual", rightPower - correction);
        if(abs(avgDist) > 0.5*abs(totalDist)){
            leftTotal += leftPower + correction;
            rightTotal += rightPower - correction;
            count ++;
        }
        SetPower(leftPower + correction, rightPower - correction);
    }

    dash->PutNumber("Left Average", leftTotal/count);
    dash->PutNumber("Right Average", rightTotal/count);
    SetPower(0.0,0.0);
}
ctre::phoenix::motorcontrol::can::WPI_TalonSRX* Tankdrive::GetTalonSRX(){
    return LeftBackDrive;
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
        double avgDist = GetEncoderDist(DriveSide::left)/2.0+GetEncoderDist(DriveSide::right)/2.0;
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
