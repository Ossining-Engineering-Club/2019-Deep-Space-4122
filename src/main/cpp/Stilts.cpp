#include "Stilts.h"

Stilts::Stilts():
    FrontStilts(0),
    RearStilts(1)
{
    StiltDrive = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(5);
    frontLimitSwitch = new frc::DigitalInput(7);
    rearLimitSwitch = new frc::DigitalInput(8);
}
void Stilts::SetFrontPower(double power){
    if(power > 0.0 && !frontLimitSwitch->Get())
        FrontStilts.Set(0.0);
    else{
        FrontStilts.Set(-1.0*power);
    }
}
void Stilts::SetRearPower(double power){
    if(power > 0.0 && !rearLimitSwitch->Get())
        RearStilts.Set(0.0);
    else{
        RearStilts.Set(power);
    }
}   
void Stilts::SetDrivePower(double power){
    StiltDrive->Set(power);
}