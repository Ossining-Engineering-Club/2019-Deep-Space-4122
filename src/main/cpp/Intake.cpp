#include "Intake.h"

Intake::Intake(){
    intakeMotor = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(11);
}
void Intake::SetPower(double power){
    intakeMotor->Set(power);
}
