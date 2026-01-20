#include "ctre/Phoenix.h"

class driveTrain {
public:
  driveTrain();
  void tankDrive(double fwd, double rot);

private:
  ctre::phoenix::motorcontrol::can::VictorSPX frontLeftMotor;
  ctre::phoenix::motorcontrol::can::VictorSPX backLeftMotor;
  ctre::phoenix::motorcontrol::can::VictorSPX frontRightMotor;
  ctre::phoenix::motorcontrol::can::VictorSPX backRightMotor;
};




// VictorSPX frontLeftMotor{1};



// // using namespace ctre::phoenix::motorcontrol;

// // can::BaseMotorController{1, "Victor SPX", can};

// // ctre::phoenix::motorcontrol::can::WPI_VictorSPX frontRightMotor{1};

// // frc::DifferentialDrive DriveTrain;
// // can::VictorSPX frontRightMotor{0};
// // can::VictorSPX frontLeftMotor{1};
// // can::VictorSPX backRightMotor{2};
// // can::VictorSPX backLeftMotor{3};

// // frc::MotorControllerGroup rightDrive;
// // frc::MotorControllerGroup LeftDrive;

// // backLeftMotor.Follow(frontLeftMotor);