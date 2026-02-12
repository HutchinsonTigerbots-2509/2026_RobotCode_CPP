// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "iostream"
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/print.h>
#include <frc/Joystick.h>
#include <frc2/command/PrintCommand.h>


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
  //double x=LimelightHelpers::getTX("limelight-b");
  //limelight_b.Clear();
}

void Robot::TeleopPeriodic() {
  double forward = -driverController.GetLeftY();
  double rotation = driverController.GetRightX();
  double trigger = driverController.GetRightTriggerAxis();
  double lTrigger = driverController.GetLeftTriggerAxis();
  double distance;
  //double distance =getDistanceFromHub("limelight-b");
  double tv=LimelightHelpers::getTV("limelight-b");
  //double distance=LimelightHelpers::getFiducialID("limelight-b");
  Launcher.Launch(trigger);
  DriveTrain.tankDrive(forward, rotation);
  frc::SmartDashboard::PutBoolean("TV",tv);
  //frc::SmartDashboard::PutNumber("power", distance);
  if(lTrigger>=.75){
    //Launcher.Launch(getDistanceFromHub("limelight-b"));
    //LimelightHelpers::LimelightResultsClass limelightResult = LimelightHelpers::getLatestResults("limelight-b");
    frc::SmartDashboard::PutNumber("y",getDistanceFromHub("limelight-b"));
    //LimelightHelpers::
    //distance = LimelightHelpers::getFiducialID("limelight-b");
    //frc::SmartDashboard::PutNumber("power",distance);
    
    
    //std::cout << std::to_string(getDistanceFromHub("limelight-b@2"));
    //frc2::PrintCommand(std::to_string(getDistanceFromHub("limelight_b")));
    //sleep(1);
  }
  
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
