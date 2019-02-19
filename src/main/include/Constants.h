#define DEBUG true

//Tankdrive stuff:

#define LEFT_ENCODER_CONST 0.0649026
#define RIGHT_ENCODER_CONST 0.0649026
#define DRIVEBASE_WIDTH 28.64

#define STRAIGHT_DRIVE_CORRECTION .935
#define BACKWARD_DRIVE_CORRECTION 0.0

#define STOP_DIST 18.0
#define STOP_POWER 0.2

#define TURN_GAMMA 1.0//2.25

//PID Constants:
    //Encoder curved driving:
#define CURVED_KP -0.2//-0.5
#define CURVED_KI 0.0
#define CURVED_KD -0.1
#define CURVED_CORRECTION 1.0

    //Gyro curved driving:
#define GYRO_KP -0.07
#define GYRO_KI -0.00000005
#define GYRO_KD -0.5
#define GYRO_CORRECTION 1.0
    //Straight driving
#define STRAIGHT_KP 0.0
#define STRAIGHT_KI 0.0
#define STRAIGHT_KD 0.0
#define STRAIGHT_CORRECTION 1.0
    //Pivots
#define TURN_KP -0.2
#define TURN_KI 0.0
#define TURN_KD -0.1
#define TURN_ACCURACY 1.5

//Utilities:
#define PI 3.1415926535897932384626433832795