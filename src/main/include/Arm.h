#pragma once
#include <rev/CANSparkMax.h>
#include <frc/WPIlib.h>
#include "Constants.h"

class Arm{
    private:
        rev::CANSparkMax *armMotor;
        rev::CANEncoder *armEncoder;
        frc::SmartDashboard *smartdash;
    public:
        Arm(frc::SmartDashboard *dash);
        void SetPower(double power, bool override);
        void SetToPosition(double power, double EncoderPosition);
        double GetEncoderPosition();
        void ResetEncoder();
};