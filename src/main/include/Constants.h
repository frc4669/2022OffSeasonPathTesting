// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#include <units/units.h>

#define AUTO_TRAJECTORY "TestPath3"

namespace DriveConstants {
    constexpr int kLeftMain = 50;
    constexpr int kLeftSlave = 51;
    constexpr int kRightMain = 52;
    constexpr int kRightSlave = 53;

    constexpr auto kTrackWidth = 18_in;

    constexpr auto ks = 0.63936_V;
    constexpr auto kv = 1.8032_V * 1_s / 1_m;
    constexpr auto ka = 0.11319_V * 1_s * 1_s / 1_m;

    constexpr double kp = 39.174;
    constexpr double ki = 0;
    constexpr double kd = 2.8251;

    // use sysid "velocity" values for this, please, not "position", 39.174 is obviously overkill

    constexpr double kWheelDiameter = 6;
    constexpr double kWheelCirc = kWheelDiameter * units::constants::pi;

    constexpr double kFirstGearRatio = (double) 30/11;
    constexpr double kSecondGearRatio = (double) 44/30;
    constexpr double kLowGearRatio = (double) 50/14;
    constexpr double kLastGearRatio = (double) 40/34;

    constexpr double kTicksPerRev = 2048;

    constexpr double kInchesPerTicksLowGear = kWheelCirc / (kTicksPerRev * kFirstGearRatio * kSecondGearRatio * kLowGearRatio * kLastGearRatio);
    constexpr double kTicksPerInchesLowGear = (kTicksPerRev * kFirstGearRatio * kSecondGearRatio * kLowGearRatio * kLastGearRatio) / kWheelCirc;

    constexpr auto kMaxAutoAccel = 1_mps_sq;
    constexpr auto kMaxAutoSpeed = 1_mps;
}