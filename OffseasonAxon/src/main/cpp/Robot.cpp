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
#include <frc/smartdashboard/SmartDashboard.h>
#include "cameraserver/CameraServer.h"
#include "NetworkTables/NetworkTable.h"
#include <frc/drive/DifferentialDrive.h>
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/Solenoid.h>


TalonSRX srx = {0};

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

// Cargo Intake Motor
WPI_VictorSPX CargoIntakeMotor{4};

// Limit Switches
frc::DigitalInput ElevatorLimitBottom{0}; 
frc::DigitalInput HatchLimitLeft{1};
frc::DigitalInput HatchLimitRight{2};

// Pneumatics
frc::Solenoid CargoIntake{0};
bool cargoButton = false;
frc::Solenoid HatchIntake{1};
bool hatchButton = false;

//Joysticks, RaceWheel, and Xbox Controller
frc::Joystick JoyAccel1{0}, Xbox{1}, RaceWheel{2};

void Robot::RobotInit() {
  //Limelight
  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  srx.Set (ControlMode::PercentOutput, 0);

  //Inverted Drive Train Motors
  LeftMotorOne.SetInverted(true);
  LeftMotorTwo.SetInverted(true);
  LeftMotorThree.SetInverted(true);  
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

  /*auto inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
  double fractionAwayFromTarget = targetOffsetAngle_Horizontal/54;

  double numberOfTargets = table->GetNumber("tv", 0.0);

  if (numberOfTargets > 0){
    LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, fractionAwayFromTarget);
    LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, fractionAwayFromTarget);
    LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, fractionAwayFromTarget);
    RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -fractionAwayFromTarget);
    RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -fractionAwayFromTarget);
    RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -fractionAwayFromTarget);
  } else {
    LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  }*/

  
  //Information to be printed
  frc::SmartDashboard::PutNumber("RightMotorEncoder: ", RightMotorOne.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("LeftMotorThree: ", LeftMotorThree.GetSelectedSensorPosition());
  /*frc::SmartDashboard::PutNumber("HatchLimitLeft:", HatchLimitLeft.Get());
  frc::SmartDashboard::PutNumber("HatchLimitRight:", HatchLimitRight.Get());
  frc::SmartDashboard::PutNumber("HatchIntake:", HatchIntake.Get());
  frc::SmartDashboard::PutNumber("ElevatorLimitBottom:", ElevatorLimitBottom.Get());*/

  // Elevator Limit Switch
  /*if(!ElevatorLimitBottom.Get()) {
    ElevatorMotorOne.SetSelectedSensorPosition(0);
  }

  //Manual Elevator Movement
  if (XboxRightAnalogY > 0.15 || XboxRightAnalogY < -0.15) {
    ElevatorMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, XboxRightAnalogY * 0.5);
    ElevatorMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, XboxRightAnalogY * 0.5);
  } else {
    ElevatorMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.15);
    ElevatorMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.15);
  }

  //Intakes
  //Cargo
  if (Xbox.GetRawButton(3) && !Xbox.GetRawButton(1)){
    CargoIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.8);
  } else if (Xbox.GetRawButton(1)){
    CargoIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
  } else {
    CargoIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.2);
  }

  //Cargo Lift
  if(Xbox.GetRawButtonPressed(2)){
    CargoIntake.Set(!CargoIntake.Get());
  }
  
  //Hatch Intake 
  if(JoyAccel1.GetRawButtonPressed(3)){
    HatchIntake.Set(!HatchIntake.Get());        
  }
  else if ((!HatchLimitLeft.Get() || !HatchLimitRight.Get()) && !HatchIntake.Get() && !JoyAccel1.GetRawButton(3)){
    HatchIntake.Set(true);
  }*/

  //Makes one 360 degree rotation
  if (JoyAccel1.GetRawButtonPressed(1)){
    LeftMotorThree.SetSelectedSensorPosition(0);
    RightMotorOne.SetSelectedSensorPosition(0);
  }
  else if (JoyAccel1.GetRawButton(1) && LeftMotorThree.GetSelectedSensorPosition() < 24000 && RightMotorOne.GetSelectedSensorPosition() < 24000){
    LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.25);
    LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.25);
    LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.25);
    RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.25);
    RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.25);
    RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.25);
  } else {
    LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  }

  //Drive Code
  //Point Turning
  /*if (RaceWheel.GetRawButton(5)) {
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
  }*/
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
    /*if (LeftMotorThree.GetSelectedSensorPosition() < 4096){
      LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.1);
      LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.1);
      LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.1);        
      RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.1);
      RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.1);
      RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.1);
    }
    else {
      LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);         
      RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    }*/
  }
}

void Robot::TeleopInit() {
  //Pneumatic Intakes
  CargoIntake.Set(false);
  HatchIntake.Set(false);

  //Elevator Motor
  ElevatorMotorOne.SetSelectedSensorPosition(0.0);

  //Drivetrain Motor Encoders
  LeftMotorThree.SetSelectedSensorPosition(0);
  RightMotorOne.SetSelectedSensorPosition(0);
  ElevatorMotorOne.SetSelectedSensorPosition(0);
}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
