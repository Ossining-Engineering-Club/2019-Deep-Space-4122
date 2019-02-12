#include "OECPIDController.h"

//Gains:
double pGain;
double iGain;
double dGain;

double max;                         // Maximum correction value

//Variables to store the different correction components:
double propCorrection = 0.0;
double intCorrection = 0.0;
double diffCorrection = 0.0;

double lastTime = 0.0;              // Time at last loop in milliseconds
double lastError = 0.0;             // Error at last loop

double isFirstRun = true;

OECPIDController::OECPIDController(double kP, double kI, double kD, double maxCorrection):
pidTimer()
{
    pGain = kP;
    iGain = kI;
    dGain = kD;
    max = maxCorrection;
}

void OECPIDController::SetPIDConstants(double kP, double kI, double kD, double maxCorrection)
{
    pGain = kP;
    iGain = kI;
    dGain = kD;
    max = maxCorrection;
}
void OECPIDController::ResetIntegral(){
    double intCorrection = 0.0;
}
double OECPIDController::GetPIDCorrection(double error)
{
    // Special case for first run: performs necessary setup steps and returns 0 correction:
    if(isFirstRun)
    {
        pidTimer.Start();
        lastTime = GetTimeMillis();
        lastError = error;
        isFirstRun = false;
        return 0.0;
    }

    propCorrection = pGain * error;
    diffCorrection = dGain * ((error-lastError)/(GetTimeMillis()-lastTime));

    //Don't increase integral component if correction is already at max.
    if(propCorrection + diffCorrection + intCorrection < max)
    {
        intCorrection += iGain * ((GetTimeMillis()-lastTime)*error);
    }
    
    //Set up for the next loop
    lastError = error;
    lastTime = GetTimeMillis();

    return propCorrection + intCorrection + diffCorrection;

}

double OECPIDController::GetTimeMillis()
{
    return pidTimer.Get()*1000.0;
}