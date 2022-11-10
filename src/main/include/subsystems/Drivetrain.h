// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/ADIS16470_IMU.h>

#include <ctre/Phoenix.h>
#include <Constants.h>

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();

  void Periodic() override;

  void CurvatureDrive(double fwd, double rot, bool fromController = false);
  
  void ResetLeftEncoder();
  void ResetRightEncoder();
  void ResetEncoders();

  frc::Field2d& GetField();

  frc::Trajectory GetAutoTrajectory();

  void ResetOdometry(frc::Pose2d pose, frc::Rotation2d angle);

  void SetVoltages(units::volt_t left, units::volt_t right);

  frc::Rotation2d GetRotation();

  frc::Rotation2d GetAutoInitialRotation();
  frc::Pose2d GetAutoInitialPose();

  frc::RamseteController& GetRamseteController();

  frc::DifferentialDriveKinematics& GetKinematics();

  units::meter_t GetLeftDistanceMeters();
  units::meter_t GetRightDistanceMeters();

  double GetLeftVelocity();
  double GetRightVelocity();

  units::meters_per_second_t GetLeftVelMetersPerSecond();
  units::meters_per_second_t GetRightVelMetersPerSecond();

  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  frc::Pose2d GetCurrentPose();

  void SetLeftVoltage(units::volt_t voltage);
  void SetRightVoltage(units::volt_t voltage);

  double GetLeftEncoderDistance(); 
  double GetRightEncoderDistance();

  units::degree_t GetHeading(); 

  frc::SimpleMotorFeedforward<units::meters> GetFeedforward();

  frc::ADIS16470_IMU& GetIMU();

 private:

  WPI_TalonFX m_leftMain { DriveConstants::kLeftMain };
  WPI_TalonFX m_leftSlave { DriveConstants::kLeftSlave };
  WPI_TalonFX m_rightMain { DriveConstants::kRightMain };
  WPI_TalonFX m_rightSlave { DriveConstants::kRightSlave };

  frc::MotorControllerGroup m_left { m_leftMain, m_leftSlave };
  frc::MotorControllerGroup m_right { m_rightMain, m_rightSlave };

  frc::DifferentialDrive m_drive { m_left, m_right };

  frc::DifferentialDriveKinematics m_kinematics { DriveConstants::kTrackWidth };
  frc::RamseteController m_ramseteController;
  frc::DifferentialDriveOdometry m_odometry { frc::Rotation2d(), frc::Pose2d() };
  frc::SimpleMotorFeedforward<units::meters> m_feedforward { DriveConstants::ks, DriveConstants::kv, DriveConstants::ka };

  frc::Field2d m_field;

  frc::ADIS16470_IMU m_imu { };

  void ConfigureMotor(WPI_TalonFX &motor, bool inverted);
};
