#include <frc/WPIlib.h>
#include <ctre/Phoenix.h>

class Stilts{
    private:
        frc::VictorSP FrontStilts;
        frc::VictorSP RearStilts;
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX *StiltDrive;
        frc::DigitalInput *frontLimitSwitch;
        frc::DigitalInput *rearLimitSwitch;
    public:
        Stilts();
        void SetFrontPower(double power);
        void SetRearPower(double power);
        void SetDrivePower(double power);

};