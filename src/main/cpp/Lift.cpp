#include "Lift.h"

Lift::Lift(frc::SmartDashboard *dash){
    LiftBackMotor = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(7);
    LiftFrontMotor = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(8);
    smartDash = dash;
    liftEncoder = new frc::Encoder(5,6,false, frc::CounterBase::EncodingType::k4X);
    liftEncoder->SetDistancePerPulse(1.0);
    liftEncoder->Reset();
}
void Lift::SetPower(double power){
    LiftFrontMotor->Set(power);
    LiftBackMotor->Set(-1.0*power);
    smartDash->PutNumber("Lift Encoder Position", liftEncoder->GetDistance());
}