#include <frc/WPIlib.h>

class Stilts{
    private:
        frc::VictorSP *FrontStilts;
        frc::VictorSP *RearStilts;
    public:
        Stilts();
        void SetFrontPower(double power);
        void SetRearPower(double power);

};