#include <subsystems/driveTrain.h>

driveTrain::driveTrain()
  : frontRightMotor(1),
    backRightMotor(2),
    frontLeftMotor(3),
    backLeftMotor(4)
{
  backLeftMotor.Follow(frontLeftMotor);
  backRightMotor.Follow(frontRightMotor);
}

void driveTrain::tankDrive(double fwd, double rot) {
  frontLeftMotor.Set(ControlMode::PercentOutput, fwd + rot);
  frontRightMotor.Set(ControlMode::PercentOutput, -(fwd - rot));
}