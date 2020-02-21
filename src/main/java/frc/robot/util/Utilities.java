/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import frc.robot.Constants;

public class Utilities {

  public static double conformAngle(double angle) {
    while (angle < 0) {
      angle += 360;
    }
    return angle % 360;
  }

  public static double deadzone(double input) {
    return deadzone(input, Constants.DEFAULT_DEADZONE);
  }

  public static double deadzone(double input, double deadzone) {
    return Math.abs(input) < Math.abs(deadzone) ? 0 : input;
  }

  /**
   * Clips the input to a maximum or minimum if the input is out of bounds
   *
   * @param input The input to clip
   * @param min   The lower bound
   * @param max   The upper bound
   * @return The clipped input
   */
  public static double clip(double input, double min, double max) {
    return Math.max(min, Math.min(input, max));
  }

  /**
   * Gets a turret hood angle from a distance to the limelight target
   * @param distance the distance to the limelight target in inches
   * @return the desired hood angle in degrees
   */
  public static double distanceToHoodAngle(double distance) {
    return 0;
  }

  /**
   * Gets a shooter wheel speed from a distance to the limelight target
   * @param distance the distance to the limelight target in inches
   * @return the desired wheel speed in rpm
   */
  public static double distanceToSpeed(double distance) {
    return 0;
  }
}
