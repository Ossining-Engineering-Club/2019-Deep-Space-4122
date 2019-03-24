#include "OECJoystick.h"

OECJoystick::OECJoystick(int port){
    stick = new frc::Joystick(port);
    for(int x = 0; x<=10; x++){
        buttons[x] = new frc::JoystickButton(stick, x+1);
    }
}
double OECJoystick::GetX(){
    return stick->GetX();
}
double OECJoystick::GetY(){
    double yAxis = stick->GetY();
    if(fabs(yAxis) > 0.055)
        return yAxis;
    else
        return 0.0;
}
double OECJoystick::GetZ(){
    return stick->GetZ();
}
bool OECJoystick::GetButton(int buttonNum){
    return buttons[buttonNum-1]->Get();
}