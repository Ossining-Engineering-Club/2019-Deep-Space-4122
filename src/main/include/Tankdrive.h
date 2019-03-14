#pragma once
#include <frc/WPIlib.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include "OECPigeonIMU.h"
#include "OECPIDController.h"
#include <ctre/Phoenix.h>
#include "Vision.h"
using namespace frc;
class Tankdrive{
    public:
        enum LoopMode{internal = 0, external = 1};
        enum DriveSide{left = 0, right = 1};

        Tankdrive(int leftPort, int rightPort, int leftEncoder1, int leftEncoder2, int rightEncoder1, int rightEncoder2, SmartDashboard *dash);
        OECPigeonIMU* GetPigeonIMU();
        void DriveStraightGyro(double power, double distInches, double startupTime, bool stopAtEnd);
        void TurnToHeading(double maxPower, double headingDegrees);
        void SetPower(double leftPower, double rightPower);
        void SetThrottle(double throttle);
        void ResetEncoders();
        void SetLoopMode(LoopMode mode);
        double GetRightEncoderDist();
        int GetRightEncoderRaw();
        double GetLeftEncoderDist();
        int GetLeftEncoderRaw();
        void AlignRobotVision(double currentDist, double targetAngle);
        void TurnToTarget(double power);
        void DriveCurveEncoder(double radius, double degrees, double avgPower, double startupTime, bool stopAtEnd);
        void DriveGyro(double degreesPerInch, double degrees, double avgPower, double timeoutSec);
        void DriveGyroByRadius(double radius, double degrees, double avgPower, double timeoutSec);
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX* GetTalonSRX();
    private:
        Vision vision;
        SmartDashboard *dash;
        OECPIDController *pidController;
        OECPigeonIMU *pigeonIMU;
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX *IMUTalonSRX;
        rev::CANSparkMax *LeftFrontDrive;
        rev::CANSparkMax *LeftBackDrive;
        rev::CANSparkMax *RightFrontDrive;
        rev::CANSparkMax *RightBackDrive;
        rev::CANEncoder *leftNeoEncoder();
        rev::CANEncoder *rightNeoEncoder();
        Encoder *leftEncoder;
        Encoder *rightEncoder;
        double throttle;
        Timer *myTimer;
        LoopMode loopMode;
            
};