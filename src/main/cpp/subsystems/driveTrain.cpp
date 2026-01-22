#include <subsystems/driveTrain.h>

driveTrain::driveTrain()
  : frontRightMotor(1),
    rearRightMotor(2),
    frontLeftMotor(3),
    rearLeftMotor(4)
{
  rearLeftMotor.Follow(frontLeftMotor);
  rearRightMotor.Follow(frontRightMotor);
}

void driveTrain::tankDrive(double fwd, double rot) {
  frontLeftMotor.Set(ControlMode::PercentOutput, fwd + rot);
  frontRightMotor.Set(ControlMode::PercentOutput, -(fwd - rot));
}
void driveTrain::tankDrive(double fwd) {     //I have this here so that the robot only moves forwards with minimal to zero rotating
  frontLeftMotor.Set(ControlMode::PercentOutput, fwd);
  frontRightMotor.Set(ControlMode::PercentOutput, fwd);
}