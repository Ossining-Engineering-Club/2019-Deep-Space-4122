#include "Intake.h"

Intake::Intake(){
    intakeMotor = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(11);
    IntakeLimSwitch = new frc::DigitalInput(7);
}
void Intake::SetPower(double power){
    if(IntakeLimSwitch->Get() && power > 0.0)
        intakeMotor->Set(0.0);
    else
        intakeMotor->Set(power);
}