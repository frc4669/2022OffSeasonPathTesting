// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * ?: Written by FRC team 4669 for the 2021-2022 season. 
 * 
 * !: Please make sure that the controller is set to XInput and not DirectInput. 
 * To check, look on the back of the controller at the X/D switch, make sure its towards X.
 * 
 * !: Please make sure that the controller mode is not on. 
 * To check, look at the controller, near the Logitech logo there is a LED, if on, press the MODE button.
 * 
 * !: The F310 controller does not have rumble. 
 * Do not use the rumble feature. It will not work.
 */

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>

class GamepadF310 {
 public:
  /**
   * Actual controller object.
   * 
   * !: Make sure the USB order ID is correct. 
   * To check, open FRC driver station and check the "USB Devices" tab and look at the order ID.
   * 
   * For example, this ID is 0. https://i.imgur.com/8G8JkBM.png 
   * For example, this ID is 3. https://i.imgur.com/daCHBMz.png
   * 
   * @param controllerID USB order ID of the controller.
   * @param leftJoyYDeadzone The deadzone for the left joystick Y axis. Default is 0.0.
   * @param leftJoyXDeadzone The deadzone for the left joystick X axis. Default is 0.0.
   * @param rightJoyYDeadzone The deadzone for the right joystick Y axis. Default is 0.0.
   * @param rightJoyXDeadzone The deadzone for the right joystick X axis. Default is 0.0.
   * @param leftTriggerDeadzone The deadzone for the left trigger. Default is 0.0.
   * @param rightTriggerDeadzone The deadzone for the right trigger. Default is 0.0.
   */
  GamepadF310(int controllerID, double leftJoyXDeadzone=0.0, double leftJoyYDeadzone=0.0, double rightJoyXDeadzone=0.0, double rightJoyYDeadzone=0.0, double leftTriggerDeadzone=0.0, double rightTriggerDeadzone=0.0);

  double getLeftJoyY();
  double getLeftJoyX();

  double getRightJoyY();
  double getRightJoyX();

  double getLeftTrigger();
  double getRightTrigger();

  
  //*: Objects 
  frc::Joystick f310Controller;

  // Colour buttons
  frc2::JoystickButton greenButton{ &f310Controller, 1 };
  frc2::JoystickButton redButton{ &f310Controller, 2 };
  frc2::JoystickButton blueButton{ &f310Controller, 3 };
  frc2::JoystickButton orangeButton{ &f310Controller, 4 };

  // Shoulder buttons
  frc2::JoystickButton leftShoulderButton{ &f310Controller, 5 };
  frc2::JoystickButton rightShoulderButton{ &f310Controller, 6 };

  // Misc buttons
  frc2::JoystickButton backButton{ &f310Controller, 7 };
  frc2::JoystickButton startButton{ &f310Controller, 8 };

  // Joystick buttons (joysticks can be pressed down)
  frc2::JoystickButton leftJoyButton{ &f310Controller, 9 };
  frc2::JoystickButton rightJoyButton{ &f310Controller, 10 };

  // POV Buttons (the D-Pad buttons that are on the left side of the controller, opposite of the colour buttons)
  frc2::POVButton dpadUpButton{ &f310Controller, 0 };
  frc2::POVButton dpadUpperRightButton{ &f310Controller, 45 };
  frc2::POVButton dpadRightButton{ &f310Controller, 90 };
  frc2::POVButton dpadLowerRightButton{ &f310Controller, 135 };
  frc2::POVButton dpadDownButton{ &f310Controller, 180 };
  frc2::POVButton dpadLowerLeftButton{ &f310Controller, 225 };
  frc2::POVButton dpadLeftButton{ &f310Controller, 270 };
  frc2::POVButton dpadUpperLeftButton{ &f310Controller, 315 };
  frc2::POVButton dpadCenteredButton{ &f310Controller, -1 };

 private:
  // Deadzone thresholds
  double leftJoyXDeadzone;
  double leftJoyYDeadzone;
  double rightJoyXDeadzone;
  double rightJoyYDeadzone;
  double leftTriggerDeadzone;
  double rightTriggerDeadzone;

  /**
   * Get value of a given axis after applying a deadzone.
   * 
   * @param axisID ID of the axis to get the value of.
   * @param threshold The deadzone threshold.
   * @return value of axis, if it's above the threshold (deadzone), 0.0 otherwise
   */
  double deadzone(int axisID, double threshold);
};
