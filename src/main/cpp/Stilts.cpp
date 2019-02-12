#include "Stilts.h"

Stilts::Stilts(){
    FrontStilts = new frc::VictorSP(0);
    RearStilts = new frc::VictorSP(1);
}
void Stilts::SetFrontPower(double power){
    FrontStilts->Set(power);
}
void Stilts::SetRearPower(double power){
    RearStilts->Set(power);
}