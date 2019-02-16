#include "Arm.h"
#include "frc/WPILib.h"
#include <iostream>

Arm::Arm(frc::SmartDashboard *dash){
    armMotor = new rev::CANSparkMax(10, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    armMotor->Set(0.0);
    armMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    armEncoder = new rev::CANEncoder(*armMotor);
    smartdash = dash;
    smartdash->PutNumber("Encoder Position", armEncoder->GetPosition());
}
void Arm::SetPower(double power, bool override){
    smartdash->PutNumber("arm power", power);
    smartdash->PutNumber("Encoder Position", armEncoder->GetPosition());
    if(armEncoder->GetPosition() <= 0.0 && power >= 0.0)
        armMotor->Set(power);
    else if(armEncoder->GetPosition() >= 0.0 && power <= -76.8)
        armMotor->Set(power);
    else if(armEncoder->GetPosition() >= -76.8 && armEncoder->GetPosition() <= 0.0){
        armMotor->Set(power);
    }
    else if(!override)
        armMotor->Set(0.0);
    else
        armMotor->Set(power);
}