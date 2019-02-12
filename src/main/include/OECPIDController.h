#pragma once
#include <frc/WPIlib.h>

class OECPIDController{
    public:
        OECPIDController(double kP, double kI, double kD, double maxCorrection);
        void SetPIDConstants(double kP, double kI, double kD, double maxCorrection);
        void ResetIntegral();
        double GetPIDCorrection(double error);
    private:
        double GetTimeMillis();
        frc::Timer pidTimer;
};