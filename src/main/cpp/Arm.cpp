#include "Arm.h"
#include "frc/WPILib.h"
#include <iostream>

Arm::Arm(frc::SmartDashboard *dash){
    smartdash = dash;
    smartdash->PutString("Init Status", "Starting Arm Encoder Initialization");
    armMotor = new rev::CANSparkMax(40, rev::CANSparkMax::MotorType::kBrushless);
    armMotor->GetEncoder();
    armMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    armEncoder = new rev::CANEncoder(*armMotor, rev::SensorType::kHallSensor, 1);
}
void Arm::SetPower(double power, bool override){
   if(!override && GetEncoderPosition() >= 0.0 && power >= 0.0)
        armMotor->Set(0.0);
    else if(!override && GetEncoderPosition() <= -87.8 && power <= 0.0)
        armMotor->Set(0.0);
    else
        armMotor->Set(power);

    smartdash->PutNumber("Arm Position", GetEncoderPosition());
    }
void Arm::SetToPosition(double power, double EncoderPosition){
    double correction = ARM_PROP*(EncoderPosition-GetEncoderPosition());
    if(abs(correction) > power)
        correction = power * (abs(correction)/correction);
    SetPower(correction, false);
    if(GetEncoderPosition() > EncoderPosition+ARM_ACCURACY){
        SetPower(-1.0 * power, false);
    }
    else if(GetEncoderPosition() < EncoderPosition - ARM_ACCURACY){
        SetPower(power, false);
    }
    else{
        SetPower(0.0, false);
    }
}
double Arm::GetEncoderPosition(){
    return armEncoder->GetPosition();
}

void Arm::ResetEncoder(){
    armEncoder->SetPosition(0.0);
}