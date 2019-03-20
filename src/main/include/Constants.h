#define DEBUG true

//Tankdrive stuff:
#define VERSION_NUMBER "1.0"

#define ARM_ACCURACY 1.5
#define LEFT_ENCODER_CONST -1.76
#define RIGHT_ENCODER_CONST 1.76
#define DRIVEBASE_WIDTH 22.75

#define STRAIGHT_DRIVE_CORRECTION 1.02
#define BACKWARD_DRIVE_CORRECTION 1.0

#define DRIVE_MAX_RPM 5700.0
#define DRIVE_PID_P 0.0
#define DRIVE_PID_I 0.0
#define DRIVE_PID_D 0.0

#define STOP_DIST 18.0
#define STOP_POWER 0.2


#define TURN_RADIUS_CORRECTION 0.3//2.25

//PID Constants:
    //Encoder curved driving:
#define CURVED_KP -0.4//-0.5
#define CURVED_KI -0.0
#define CURVED_KD 0.0
#define CURVED_CORRECTION 1.0

#define LIFT_TOP_POSITION 1000000.0
    //Gyro curved driving:
#define GYRO_KP -0.07
#define GYRO_KI -0.00000005
#define GYRO_KD -0.5
#define GYRO_CORRECTION 1.0

#define VISION_P 0.0007
#define VISION_I 0.0
#define VISION_D 0.0
    //Straight driving
#define STRAIGHT_KP 0.0
#define STRAIGHT_KI 0.0
#define STRAIGHT_KD 0.0
#define STRAIGHT_CORRECTION 1.0
    //Pivots
#define TURN_KP 0.4
#define TURN_KI 0.0
#define TURN_KD -0.2
#define TURN_ACCURACY 1.5

#define PIXEL_ACCURACY 5.0

//Utilities:
#define PI 3.1415926535897932384626433832795

//--------------------AUTONOMOUS ------------------------------------------------------------
//SIDE
#define STRAIGHT_SPEED_1 0.3
#define TURN_SPEED_1 0.3
#define STRAIGHT_SPEED_2 0.3
#define TURN_SPEED_2 0.15
#define STRAIGHT_SPEED_3 0.2
#define TURN_SPEED_3 0.3

#define SIDE_STRAIGHT_DIST_1 80.0
#define SIDE_TURN_HEADING_1 8.0
#define SIDE_STRAIGHT_DIST_2 136.0
#define SIDE_TURN_HEADING_2 93.0
#define FINAL_DISTANCE 0.0
#define FINAL_TURN_HEADING 0.0
//CENTER
#define CENTER_STRAIGHT_DIST_1 48.0
#define CENTER_TURN_HEADING_1 2.0
#define CENTER_STRAIGHT_DIST_2 63.0
