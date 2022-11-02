// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/RunCommand.h>

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here
    m_drivetrain.SetDefaultCommand(frc2::RunCommand(
    [this] 
      {  m_drivetrain.CurvatureDrive(i_f310.getLeftJoyY(), i_f310.getRightJoyX() * 0.50, true); },
      {  &m_drivetrain  }
  ));

  m_drivetrain.ResetEncoders();
  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  m_drivetrain.ResetOdometry(m_drivetrain.GetAutoInitialPose(), m_drivetrain.GetAutoInitialRotation());

  std::function<frc::Pose2d()> getPose = [this] () { return m_drivetrain.GetCurrentPose(); };
  std::function<frc::DifferentialDriveWheelSpeeds()> getWheelSpeeds = [this] () { return m_drivetrain.GetWheelSpeeds(); };
  std::function<void(units::volt_t, units::volt_t)> setVoltages = [this] (auto left, auto right) {
    m_drivetrain.SetLeftVoltage(left);
    m_drivetrain.SetRightVoltage(right);
  };

  frc2::RamseteCommand followPathplannerFile {
    m_drivetrain.GetAutoTrajectory(), //Gets the trajectory from pathplannnerlib
    getPose, //Allows the command to repeatedly retrieve the pose from the odometry
    m_drivetrain.GetRamseteController(),
    m_drivetrain.GetFeedforward(),
    m_drivetrain.GetKinematics(),
    getWheelSpeeds, //Allows the command to repeatedly get the speeds of the wheels
    frc2::PIDController(DriveConstants::kp, DriveConstants::ki, DriveConstants::kd), //PID controller
    frc2::PIDController(DriveConstants::kp, DriveConstants::ki, DriveConstants::kd), //PID controller
    setVoltages, //Sets voltage of motors based on command output
    { &m_drivetrain }
  };
  
  return new frc2::SequentialCommandGroup(
    std::move(followPathplannerFile)
  );
}