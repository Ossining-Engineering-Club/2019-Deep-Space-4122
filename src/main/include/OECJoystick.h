#pragma once
#include <frc/WPIlib.h>

class OECJoystick{
    private:
        frc::Joystick *stick;
        frc::JoystickButton *buttons[11];
    public:
        OECJoystick(int port);
        double GetX();
        double GetY();
        double GetZ();
        bool GetButton(int buttonNum);
};