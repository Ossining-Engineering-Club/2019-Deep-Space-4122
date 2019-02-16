#include "Stilts.h"

Stilts::Stilts(){
    FrontStilts = new frc::VictorSP(0);
    RearStilts = new frc::VictorSP(1);
    StiltDrive = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(5);
}
void Stilts::SetFrontPower(double power){
    FrontStilts->Set(-1*power);
}
void Stilts::SetRearPower(double power){
    RearStilts->Set(power);
}   
void Stilts::SetDrivePower(double power){
    StiltDrive->Set(power);
}