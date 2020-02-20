/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import frc.robot.Constants;

/**
 * Add your docs here.
 */
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
}
