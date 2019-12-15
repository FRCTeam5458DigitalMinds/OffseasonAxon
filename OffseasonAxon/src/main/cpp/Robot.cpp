/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <string>
#include <sstream>
#include <Robot.h>
#include <WPILib.h>
#include <stdlib.h>
#include <iostream>
#include <Encoder.h>
#include <frc/Timer.h>
#include <TimedRobot.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>

TalonSRX srx = {0};

//Motors

// Right Side Drive Motors
WPI_TalonSRX RightMotorOne{15}; // Encoder
WPI_VictorSPX RightMotorTwo{14};
WPI_VictorSPX RightMotorThree{13};
// Left Side Drive Motors
WPI_VictorSPX LeftMotorOne{2};
WPI_VictorSPX LeftMotorTwo{1};
WPI_TalonSRX LeftMotorThree{0}; // Encoder

//Elevator Motors
WPI_TalonSRX ElevatorMotorOne{12};
WPI_TalonSRX ElevatorMotorTwo{3}; // Encoder

// Limit Switches
frc::DigitalInput ElevatorLimitBottom{0};

// Pneumatics

//Joysticks, RaceWheel, and Xbox Controller
frc::Joystick JoyAccel1{0}, Xbox{1}, RaceWheel{2};

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  srx.Set (ControlMode::PercentOutput, 0);

  LeftMotorOne.SetInverted(true);
  LeftMotorTwo.SetInverted(true);
  LeftMotorThree.SetInverted(true);  

  //Elevator Motor
  ElevatorMotorOne.SetSelectedSensorPosition(0.0);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

  //Controllers For Driving and Operating
  double JoyY = -JoyAccel1.GetY();
  double WheelX = RaceWheel.GetX();
  double XboxRightAnalogY = Xbox.GetRawAxis(5);

  // Elevator Limit Switch
  if(ElevatorLimitBottom.Get()) {
    ElevatorMotorOne.SetSelectedSensorPosition(0);
  }

  //Manual Elevator Movement
  if (XboxRightAnalogY > 0.15 || XboxRightAnalogY < -0.15) {
    ElevatorMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, XboxRightAnalogY * 0.5);
    ElevatorMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, XboxRightAnalogY * 0.5);
  } else {
    ElevatorMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    ElevatorMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  }
  
  //Drive Code
  //Point Turning
  if (RaceWheel.GetRawButton(5)) {
    LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
    LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
    LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
    RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);   
  } 
  //Regular Turning
  else if((WheelX < -0.01 || WheelX > 0.01) && (JoyY > 0.06 || JoyY < -0.06)){
    LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX + JoyY);
    LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX + JoyY);
    LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX + JoyY);
    RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY - WheelX);
    RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY - WheelX);
    RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY - WheelX);
  }
  //Code for driving straight  
  else if (JoyY > 0.1|| JoyY < -0.1 ) {
    LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);
    LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);
    LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);
    RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);
    RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);
    RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);                  
  } 
  //No Joystick Input
  else {
    LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);        
    RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  }

}

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
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
