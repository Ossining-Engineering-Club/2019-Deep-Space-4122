#include "Arm.h"
#include "frc/WPILib.h"
#include <iostream>

Arm::Arm(frc::SmartDashboard *dash){
    armMotor = new rev::CANSparkMax(10, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    armMotor->Set(0.0);
    armMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    armEncoder = new rev::CANEncoder(*armMotor);
    smartdash = dash;
}
void Arm::SetPower(double power, bool override){
    if(!override && armEncoder->GetPosition() >= 0.0 && power >= 0.0)
        armMotor->Set(0.0);
    else if(!override && armEncoder->GetPosition() <= -87.8 && power <= 0.0)
        armMotor->Set(0.0);
    else
        armMotor->Set(power);

    smartdash->PutNumber("Arm Position", armEncoder->GetPosition());
}
void Arm::SetToPosition(double power, double EncoderPosition){
    if(GetEncoderPosition() > EncoderPosition-ARM_ACCURACY){
        SetPower(-1.0 * power, false);
    }
    else if(GetEncoderPosition() < EncoderPosition + ARM_ACCURACY){
        SetPower(power, false);
    }
    else{
        SetPower(0.0, false);
    }
}
double Arm::GetEncoderPosition(){
    return armEncoder->GetPosition();
}