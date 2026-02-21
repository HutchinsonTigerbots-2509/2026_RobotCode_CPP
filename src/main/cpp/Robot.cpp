// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"


Robot::Robot() 
: driverController{0}
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}
/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  wpi::print("Auto selected: {}\n", m_autoSelected);
  double forward = 0.25;
  double rotation = 0;
  
  if (m_autoSelected == kAutoNameCustom) {
    DriveTrain.tankDrive(forward, rotation);
    sleep(1);
    DriveTrain.tankDrive(0, .25);
    sleep(1);
    DriveTrain.tankDrive(0,0);
    sleep(1);
    DriveTrain.tankDrive(0.25,0);
    sleep(1);
    DriveTrain.tankDrive(0,0);
  } else {
    // Default Auto goes here
    DriveTrain.tankDrive(forward, rotation);
    sleep(1);
    DriveTrain.tankDrive(0, .25);
    sleep(1);
    DriveTrain.tankDrive(0,0);
    sleep(.25);
    DriveTrain.tankDrive(0.25,0);
    sleep(1);
    DriveTrain.tankDrive(0,0);
    sleep(.25);
    DriveTrain.tankDrive(0,0.25);
    sleep(1);
    DriveTrain.tankDrive(0,0);
    sleep(.25);
    DriveTrain.tankDrive(0.25,0);
    sleep(1);
    DriveTrain.tankDrive(0,0);
    sleep(.25);
    DriveTrain.tankDrive(0,0.25);
    sleep(1);
    DriveTrain.tankDrive(0,0);
    sleep(.25);
    DriveTrain.tankDrive(0.25,0);
    sleep(1);
    DriveTrain.tankDrive(0,0);
    
  }
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  double angleDerivative=frc::SmartDashboard::PutNumber("Derivative",0);//These are tuners      it works don't question
  double angleIntegral=frc::SmartDashboard::PutNumber("Integral",0);//These are tuners          it smooths the motion on the robot.
  double angleProptional=frc::SmartDashboard::PutNumber("Proptional",0);//These are tuners
  frc::SmartDashboard::SetDefaultNumber("Derivative",0);
  frc::SmartDashboard::SetDefaultNumber("Integral",0);
  frc::SmartDashboard::SetDefaultNumber("Proptional",0);
  //double x=LimelightHelpers::getTX("limelight-b");
  //limelight_b.Clear();
}

void Robot::TeleopPeriodic() {
  double angleDerivative= 0.05;//These are tuners       it works don't question
  double angleIntegral=0;//frc::SmartDashboard::GetNumber("Integral", 0);//These are tuners          it smooths the motion on the robot.
  double angleProptional=0.05;//frc::SmartDashboard::GetNumber("Proptional", 0);//These are tuners 
  
  
  
  double distProptional=0.75; //Range of values to adjust to so increase makes output range higher and vice versa
  double distIntegral=0; //Doesn't get used very much in this situation, smooths out curve
  double distDerivative=0; //Rate of change of your proportional, smooths out the change
  std::cout << std::to_string(angleProptional);
  frc::PIDController distTargetingPID(distProptional,distIntegral,distDerivative); //PID to maintain dist to target
  frc::PIDController angleTargetingPID{angleProptional,angleIntegral,angleDerivative}; //PID to maintain angle to target
  double forward;
  double rotation;
  double trigger = driverController.GetRightTriggerAxis();
  double lTrigger = driverController.GetLeftTriggerAxis();
  double distance;
  bool vision_button = false;
  if(driverController.GetAButton()){
    frc::SmartDashboard::PutBoolean("TV", driverController.GetAButton());
    if(trackingTag(12, &distance, &rotation)){
      frc::SmartDashboard::PutNumber("Angle PID Result:", .1*angleTargetingPID.Calculate(rotation,0));
      frc::SmartDashboard::PutNumber("Distance PID Result:", distTargetingPID.Calculate(distance,1));
      DriveTrain.tankDrive(-1*distTargetingPID.Calculate(distance,1.5),-1*angleTargetingPID.Calculate(rotation,0)); // While the A button is pressed it gets your dist and angle from ID 12 to maintain 1.5m dist and 0 degree angle
      frc::SmartDashboard::PutNumber("Rotation:",rotation);
      frc::SmartDashboard::PutNumber("Distance:",distance);
      frc::SmartDashboard::PutNumber("power", angleTargetingPID.Calculate(rotation,0));
    }
    else{
      DriveTrain.tankDrive(0, 0);
    }
  }else{
    forward = -driverController.GetLeftY();
    rotation = driverController.GetRightX();
    frc::SmartDashboard::PutBoolean("TV", driverController.GetAButton());

    DriveTrain.tankDrive(0, angleTargetingPID.Calculate(0,0));
  }
  //double distance =getDistanceFromHub("limelight-b");
  //double tv=LimelightHelpers::getTV("limelight-b");
  //double distance=LimelightHelpers::getFiducialID("limelight-b");
  //Launcher.Launch(trigger);
  //DriveTrain.tankDrive(forward, rotation);
  //frc::SmartDashboard::PutBoolean("TV",tv);
  //frc::SmartDashboard::PutNumber("power", distance);
  //frc::SmartDashboard::PutNumber("tx",LimelightHelpers::getTX("limelight-b"));
  //if(a){
  //DriveTrain.aiming(getDistanceFromHub("limelight-b"));
  //frc::SmartDashboard::PutNumber("y",getDistanceFromHub("limelight-b"));
  // if(driverController.GetAButtonPressed()){vision_button = true;}
  // while(getDistanceFromHub("limelight-b") >= .1 and vision_button){
  //   std::cout << "\nPressing A\n";
  //   frc::SmartDashboard::PutNumber("y",getDistanceFromHub("limelight-b"));
  //   if(driverController.GetAButtonReleased()){vision_button = false;}
  // }
  // DriveTrain.tankDrive(forward, rotation);
  // //}
  // if(lTrigger>=.75){
  //   frc::SmartDashboard::PutNumber("y",getDistanceFromHub("limelight-b"));
  // }
  
  
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  phoenix6::SignalLogger::Stop;
  return frc::StartRobot<Robot>();
}
#endif
