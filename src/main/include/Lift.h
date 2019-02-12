#include <frc/WPIlib.h> 
#include <ctre/Phoenix.h>

class Lift{
    private:
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX *LiftBackMotor;
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX *LiftFrontMotor;

    public:
        Lift();
        void SetPower(double power);
};