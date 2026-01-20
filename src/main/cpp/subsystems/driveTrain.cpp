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