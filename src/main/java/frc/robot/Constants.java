/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double AIO_MAX_VOLTAGE = 5.0;

  public static final double DEFAULT_DEADZONE = 0.1;

  public static final class Climber {
    public static final double MAX_PERCENT_OUTPUT = 0.5;

    public static final double ENCODER_INCHES_PER_TICK = 1;
  }

  public static final class ControlPanel {
    public static final Color COLOR_BLUE = ColorMatch.makeColor(0, 1, 1);
    public static final Color COLOR_GREEN = ColorMatch.makeColor(0, 1, 0);
    public static final Color COLOR_RED = ColorMatch.makeColor(1, 0, 0);
    public static final Color COLOR_YELLOW = ColorMatch.makeColor(1, 1, 0);

    public static final double PANEL_CIRCUMFERENCE = 32 * Math.PI; // On-field mechanism. Inches
    public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI; // On-robot wheel. Inches
    public static final double ENCODER_DEGREES_PER_TICK = 360.0 / 128.0; // TODO: Check factor of 4
  }

  public static final class DriveTrain {
    public static final double TRACK_WIDTH = 0.0; // Inches
    public static final int ENCODER_RESOLUTION = 128;
    public static final double ENCODER_METERS_PER_TICK = 6d * Math.PI / (double) ENCODER_RESOLUTION;

    public static final double RAMSETE_B = 2.0;
    public static final double RAMSETE_ZETA = 0.7;

    public static final double VELOCITY_P = 0.0;
    public static final double VELOCITY_I = 0.0;
    public static final double VELOCITY_D = 0.0;
    public static final double VELOCITY_FF = 0.0;
  }

  public static final class Hood {
    public static final double POSITION_P = 20.0;
    public static final double POSITION_I = 0.0;
    public static final double POSITION_D = 0.0;
    public static final double POSITION_TOLERANCE = 2.0; // Degrees

    public static final double POT_SCALE = 3600; // Degrees (360 * Number of pot rotations)
    // TODO : Fill In This Value
    public static final double POT_DEGREES_PER_HOOD_MOVE = 1.22 * 360; // Degrees

    public static final double MIN_ANGLE = 27.4; // Degrees
    public static final double MAX_ANGLE = 82.5; // Degrees

    public static final double MAX_RAW_ANGLE = POT_SCALE - 180; // Degrees
    public static final double MIN_RAW_ANGLE = 180; // Degrees

    public static final double PWM_MAX = 2.5;
    public static final double PWM_DEADBAND_MAX = 1.55;
    public static final double PWM_CENTER = 1.5;
    public static final double PWM_DEADBAND_MIN = 1.45;
    public static final double PWM_MIN = 0.5;

    public static final double PRESET_1_ANGLE = 63.23; // Degrees for 18 Foot shot
  }

  public static final class Revolver {
    public static final double FORWARDS_SPEED = 0.05;
    public static final double BACKWARDS_SPEED = -0.05;

    public static final double VELOCITY_THRESHOLD = 60; // RPM
    public static final int STALL_REPS_THRESHOLD = 10;

    public static final double UNJAM_TIME = 0.5; // Time to unjam in seconds
    public static final double UNJAM_COOLDOWN = 0.5; // Minimum time between unjams in seconds
  }

  public static final class Shooter {
    public static final double MAX_PERCENT_OUTPUT = 0.9;
    public static final double MAX_VOLTAGE_RAMP_RATE = 0.2;

    public static final double VELOCITY_P = 0.002;
    public static final double VELOCITY_I = 0.00001;
    public static final double VELOCITY_D = 0.0;
    public static final double VELOCITY_FF = 0.000216350747274 / 2d;
    public static final double VELOCITY_TOLERANCE = 50.0; // RPM

    public static final double IDLE_SPEED = 1500; // RPM
    public static final double MAX_SPEED = 6000; // RPM

    public static final double RATIO = 2d / 1d;

    public static final double PRESET_1_SPEED = 3450; // RPM
  }

  public static final class Turret {
    public static final double MAX_PERCENT_OUTPUT = 0.9;

    public static final double POSITION_P = 0.014;
    public static final double POSITION_I = 0.0105;
    public static final double POSITION_D = 0.0002;
    public static final double POSITION_TOLERANCE = 1.0; // Degrees
    public static final double VELOCITY_TOLERANCE = 0.1; // Degrees per 100ms

    public static final double MIN_ANGLE = -90.0; // Full left
    public static final double MAX_ANGLE = 90.0; // Full right
    public static final double HOME_ANGLE = 0.0; // Center

    public static final double ANGLE_OFFSET = 121.6; // Center Offset

    public static final double POT_SCALE = 3600;

    public static final double RATIO = 18d / 265d;
  }

  public static final class Vision {
    public static final double TARGET_HEIGHT = 60.0; // Inches from ground
    public static final double INNER_HOLE_OFFSET = 29.25; // Inches begind the high target

    public static final double LIMELIGHT_HEIGHT = 39.5; // Inches from ground
    public static final double LIMELIGHT_PITCH = 29.8; // Degrees from horizontal
    public static final double LIMELIGHT_POSITION_OFFSET = 7.82326; // Inches forward from center of turret

    public static final double LIMELIGHT_HORIZONTAL_FOV = 53.0;
  }
}
