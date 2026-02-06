#include "ctre/Phoenix.h"

class driveTrain {
public:
  driveTrain();
  void tankDrive(double fwd, double rot);
  void tankDrive(double rot);

private:
  ctre::phoenix::motorcontrol::can::VictorSPX frontLeftMotor;
  ctre::phoenix::motorcontrol::can::VictorSPX rearLeftMotor;
  ctre::phoenix::motorcontrol::can::VictorSPX frontRightMotor;
  ctre::phoenix::motorcontrol::can::VictorSPX rearRightMotor;
};