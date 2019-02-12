#include "Lift.h"

Lift::Lift(){
    LiftBackMotor = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(7);
    LiftFrontMotor = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(8);
}
void Lift::SetPower(double power){
    LiftFrontMotor->Set(power);
    LiftBackMotor->Set(-1.0*power);
}