#pragma once
#include <frc/WPIlib.h>
#include <ctre/Phoenix.h>
class Intake{
    private:
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX *intakeMotor;
        frc::DigitalInput *IntakeLimSwitch;
    public:
        Intake();
        void SetPower(double power);
};