/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
}
