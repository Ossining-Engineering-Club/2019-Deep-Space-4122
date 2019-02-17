#include "Tankdrive.h"
#include <iostream>

using namespace std;
Tankdrive::Tankdrive(int leftPort, int rightPort, int leftEncoder1, int leftEncoder2, int rightEncoder1, int rightEncoder2, OECPigeonIMU *pigeonIMU, SmartDashboard *dash){
    this->dash = dash;
    this->pigeonIMU = pigeonIMU;
    pidController = new OECPIDController(CURVED_KP, CURVED_KI, CURVED_KD, CURVED_CORRECTION);
    LeftFrontDrive = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(1);
    LeftBackDrive = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(2);
    RightFrontDrive = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(3);
    RightBackDrive = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(4);
    leftEncoder = new frc::Encoder(leftEncoder1, leftEncoder2, false, CounterBase::EncodingType::k4X);
    rightEncoder = new frc::Encoder(rightEncoder1, rightEncoder2, true, CounterBase::EncodingType::k4X);
    rightEncoder->SetDistancePerPulse(ENCODER_CONST);
    leftEncoder->SetDistancePerPulse(ENCODER_CONST);
    leftEncoder->Reset();
    rightEncoder->Reset();
    throttle = 1.0;
    myTimer = new Timer();
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
double Tankdrive::GetLeftEncoderDist(){
    return leftEncoder->GetDistance();
}
double Tankdrive::GetRightEncoderDist(){
    return rightEncoder->GetDistance();
}
void Tankdrive::TurnToHeading(double maxPower, double headingDegrees){
    double turnError = 0.0;
    double correction = 0.0;
    pidController = new OECPIDController(TURN_KP, TURN_KI, TURN_KD, maxPower);
    while(abs(headingDegrees-pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees)) > TURN_ACCURACY){
        correction = pidController->GetPIDCorrection(pigeonIMU->GetYaw(OECPigeonIMU::AngleUnits::degrees)-headingDegrees);
        SetPower(correction, -1.0*correction);
    }
    SetPower(0.0, 0.0);
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
        if(abs(power/2.0) < 0.2)
            pow = (abs(power)/power)*power/2.0;
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
}
void Tankdrive::DriveCurveEncoder(double radius, double degrees, double avgPower, double startupTime, bool stopAtEnd){
    double tempPower = 0.0;
    pidController = new OECPIDController(CURVED_KP, CURVED_KI, CURVED_KD, CURVED_CORRECTION);
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
    myTimer->Stop();
    myTimer->Reset();
    myTimer->Start();
    while(abs(avgDist) < abs(totalDist)){
        if(myTimer->Get() < startupTime){
            tempPower = (myTimer->Get()/startupTime)*avgPower;
            double leftPower = tempPower * (1 - 0.5*DRIVEBASE_WIDTH/radius);
            double rightPower = tempPower * (1 + 0.5*DRIVEBASE_WIDTH/radius);
        }
        else{
            double leftPower = avgPower * (1 - 0.5*DRIVEBASE_WIDTH/radius);
            double rightPower = avgPower * (1 + 0.5*DRIVEBASE_WIDTH/radius);
        }
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
    if(stopAtEnd)
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
