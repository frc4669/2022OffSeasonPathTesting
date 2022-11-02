// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>

using namespace pathplanner;

Drivetrain::Drivetrain() {
    m_drive.SetSafetyEnabled(false);

    ConfigureMotor(m_leftMain, true);
    ConfigureMotor(m_leftSlave, true);
    m_leftSlave.Follow(m_leftMain);

    ConfigureMotor(m_rightMain, false);
    ConfigureMotor(m_rightSlave, false);
    m_rightSlave.Follow(m_rightMain);

    ResetEncoders();

    frc::SmartDashboard::PutData("Field", &m_field);
}

void Drivetrain::Periodic() {
    m_odometry.Update(GetRotation(), GetLeftDistanceMeters(), GetRightDistanceMeters());

    m_field.SetRobotPose(m_odometry.GetPose());
}

void Drivetrain::CurvatureDrive(double fwd, double rot, bool fromController) {
    m_drive.CurvatureDrive(fwd, rot, true);
}

frc::Trajectory Drivetrain::GetAutoTrajectory() {
  PathPlannerTrajectory autonomousPath = PathPlanner::loadPath(AUTO_TRAJECTORY, 4_mps, 4_mps_sq);
  frc::Trajectory trajectory = autonomousPath.asWPILibTrajectory();
  return trajectory;
}

frc::Pose2d Drivetrain::GetAutoInitialPose() {
  PathPlannerTrajectory autonomousPath = PathPlanner::loadPath(AUTO_TRAJECTORY, 4_mps, 4_mps_sq);
  return autonomousPath.getInitialState()->pose;
}

frc::Rotation2d Drivetrain::GetAutoInitialRotation() {
  PathPlannerTrajectory autonomousPath = PathPlanner::loadPath(AUTO_TRAJECTORY, 4_mps, 4_mps_sq);
  return autonomousPath.getInitialState()->pose.Rotation();
}

void Drivetrain::ResetLeftEncoder() {
  m_leftMain.GetSensorCollection().SetIntegratedSensorPosition(0);
}

void Drivetrain::ResetRightEncoder() {
  m_rightMain.GetSensorCollection().SetIntegratedSensorPosition(0);
}

double Drivetrain::GetLeftVelocity() {
  return m_leftMain.GetSensorCollection().GetIntegratedSensorVelocity();
}

double Drivetrain::GetRightVelocity() {
  return m_rightMain.GetSensorCollection().GetIntegratedSensorVelocity();
}

void Drivetrain::ResetEncoders() {
  ResetLeftEncoder();
  ResetRightEncoder();
}

double Drivetrain::GetLeftEncoderDistance() {
  return m_leftMain.GetSensorCollection().GetIntegratedSensorPosition();
}

double Drivetrain::GetRightEncoderDistance() {
  return m_rightMain.GetSensorCollection().GetIntegratedSensorPosition();
}

units::degree_t Drivetrain::GetHeading() {
  // return units::degree_t(std::remainder(m_imu.GetAngle(), 360.0)) * (DriveConstants::kGyroReversed ? -1.0 : 1.0); // !: Come back to this
  return m_imu.GetAngle();
  // return units::degree_t(0);
}

frc::Rotation2d Drivetrain::GetRotation() {
  return frc::Rotation2d(m_imu.GetAngle());
}

frc::RamseteController& Drivetrain::GetRamseteController() {
  return m_ramseteController;
}

frc::DifferentialDriveKinematics& Drivetrain::GetKinematics() {
  return m_kinematics;
}

frc::ADIS16470_IMU& Drivetrain::GetIMU() {
  return m_imu;
}

void Drivetrain::ConfigureMotor(WPI_TalonFX &motor, bool inverted) {
  // set the max velocity and acceleration for motion magic
  motor.ConfigMotionCruiseVelocity(20000);
  motor.ConfigMotionAcceleration(7000);

  // set the current limit for the supply/output current
  motor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 25, 25, 0.5));
  motor.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 25, 25, 0.5));

  // time it takes for the motor to go from 0 to full power (in seconds) in an open/closed loop
  motor.ConfigOpenloopRamp(0.2);
  motor.ConfigClosedloopRamp(0);

  // when controller is neutral, set motor to break
  motor.SetNeutralMode(NeutralMode::Brake);

  // disable motor safety
  motor.SetSafetyEnabled(false);

  // motor set experation time
  motor.SetExpiration(100_ms);

  // invert the motor if necessary
  motor.SetInverted(inverted);

  // Motor PID values (for now)
  motor.Config_kP(0, 0.01); // kP, the proportional constant (how fast the motor changes speed), acts like a “software-defined springs”
  motor.Config_kD(0, 0.00); // kD, the derivative constant (drives the velocity error to zero)
  motor.Config_kF(0, 0.00); // kF, the feed forward constant (how much the output is affected by the setpoint)
}

void Drivetrain::ResetOdometry(frc::Pose2d pose, frc::Rotation2d angle) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, angle);
}

units::meter_t Drivetrain::GetLeftDistanceMeters() {
  return units::meter_t(
    units::inch_t(
      m_leftMain.GetSensorCollection().GetIntegratedSensorPosition() * DriveConstants::kInchesPerTicksLowGear
    )
  );
}

units::meter_t Drivetrain::GetRightDistanceMeters() {
  return units::meter_t(
    units::inch_t(
      m_rightMain.GetSensorCollection().GetIntegratedSensorPosition() * DriveConstants::kInchesPerTicksLowGear
    )
  );
}

units::meters_per_second_t Drivetrain::GetLeftVelMetersPerSecond() {
  double ticksPerSecond = m_leftMain.GetSensorCollection().GetIntegratedSensorVelocity() * 10;
  double velMpS = ticksPerSecond * DriveConstants::kInchesPerTicksLowGear;

  return units::meters_per_second_t(
    units::meter_t(units::inch_t(velMpS)).value()
  );
}

units::meters_per_second_t Drivetrain::GetRightVelMetersPerSecond() {
  double ticksPerSecond = m_rightMain.GetSensorCollection().GetIntegratedSensorVelocity() * 10;
  double velMpS = ticksPerSecond * DriveConstants::kInchesPerTicksLowGear;

  return units::meters_per_second_t(
    units::meter_t(units::inch_t(velMpS)).value()
  );
}

frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds() {
  return {
    GetLeftVelMetersPerSecond(),
    GetRightVelMetersPerSecond()
  };
}

frc::Pose2d Drivetrain::GetCurrentPose() {
  return m_odometry.GetPose();
}

void Drivetrain::SetLeftVoltage(units::volt_t voltage) {
  m_leftMain.SetVoltage(voltage);
}

void Drivetrain::SetRightVoltage(units::volt_t voltage) {
  m_rightMain.SetVoltage(voltage);
}

frc::SimpleMotorFeedforward <units::meters> Drivetrain::GetFeedforward() {
  return m_feedforward;
}
