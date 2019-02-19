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
    if(armEncoder->GetPosition() <= 0.0 && power >= 0.0)
        armMotor->Set(power);
    else if(armEncoder->GetPosition() >= 0.0 && power <= -76.8)
        armMotor->Set(power);
    else if(armEncoder->GetPosition() >= -87.8 && armEncoder->GetPosition() <= 0.0){
        armMotor->Set(power);
    }
    else if(!override)
        armMotor->Set(0.0);
    else
        armMotor->Set(power);
}
double Arm::GetEncoderPosition(){
    return armEncoder->GetPosition();
}