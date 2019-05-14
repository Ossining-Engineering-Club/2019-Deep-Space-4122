#include "Lift.h"
#include "Constants.h"

Lift::Lift(frc::SmartDashboard *dash){
    LiftBackMotor = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(7);
    LiftFrontMotor = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(8);
    smartDash = dash;
    liftEncoder = new frc::Encoder(5,6,false, frc::CounterBase::EncodingType::k4X);
    liftEncoder->SetDistancePerPulse(1.0);
    liftEncoder->Reset();
}
void Lift::SetPower(double power){
    smartDash->PutNumber("Lift Position", GetEncoderPosition());
    if(GetEncoderPosition() < MINLIFT && power < 0.0){
        smartDash->PutString("Lift State", "Too Low");
        LiftFrontMotor->Set(0.0);
        LiftBackMotor->Set(0.0);
    }
    else if(GetEncoderPosition() > MAXLIFT && power > 0.0){
        smartDash->PutString("Lift State", "Too High");
        LiftFrontMotor->Set(0.0);
        LiftBackMotor->Set(0.0);
    }
    else{
        smartDash->PutString("Lift State", "Correct");
    LiftFrontMotor->Set(power);
    LiftBackMotor->Set(-1.0*power);
    }
}
double Lift::GetEncoderPosition(){
    return liftEncoder->GetDistance();
}
void Lift::SetToPosition(double power, double EncoderPosition){
    double correction = LIFT_PROP*(EncoderPosition-GetEncoderPosition());
    if(abs(correction) > power)
        correction = power * (abs(correction)/correction);
    SetPower(correction);
    /*if(GetEncoderPosition() > EncoderPosition+LIFT_ACCURACY){
        SetPower(-1.0 * power);
    }
    else if(GetEncoderPosition() < EncoderPosition - LIFT_ACCURACY){
        SetPower(power);
    }
    else{
        SetPower(0.0);
    }*/
}