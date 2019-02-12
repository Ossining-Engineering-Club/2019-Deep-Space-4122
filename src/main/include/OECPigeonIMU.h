#pragma once
#include <ctre/Phoenix.h>

class OECPigeonIMU{
    public:
        enum AngleUnits{degrees, radians};
        OECPigeonIMU(int canPort);
        OECPigeonIMU(ctre::phoenix::motorcontrol::can::TalonSRX *talonSRX);
        void ResetAngle();
        void BootCalibrate();
        double GetYaw(AngleUnits units);
        double GetPitch(AngleUnits units);
        double GetRoll(AngleUnits units);
    private:
        ctre::phoenix::sensors::PigeonIMU *pigeonGyro;
};