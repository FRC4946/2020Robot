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
    public static final double ENCODER_DEGREES_PER_TICK = 360.0 / 128.0;
  }

  public static final class DriveTrain {
    /**
     * Track width in inches
     */
    public static final double TRACK_WIDTH = 0.0;
    public static final double ENCODER_METERS_PER_TICK = 1;
    public static final double ENCODER_MPS_PER_RPM = 0.0;
    public static final int ENCODER_RESOLUTION = 128;

    public static final double RAMSETE_B = 0.0;
    public static final double RAMSETE_ZETA = 0.0;

    public static final double VELOCITY_P = 0.0;
    public static final double VELOCITY_I = 0.0;
    public static final double VELOCITY_D = 0.0;
    public static final double VELOCITY_FF = 0.0;
  }

  public static final class Hood {
    public static final double POSITION_P = 0.0;
    public static final double POSITION_I = 0.0;
    public static final double POSITION_D = 0.0;
    public static final double POSITION_TOLERANCE = 2.0; // Degrees

    public static final double POT_SCALE = 5; // TODO: Dummy value
    public static final double POT_OFFSET = 1; // TODO: Dummy value

    public static final double MIN_ANGLE = 27.4; // Degrees
    public static final double MAX_ANGLE = 82.5; // Degrees

    public static final double PWM_MAX = 2.5;
    public static final double PWM_DEADBAND_MAX = 2.45;
    public static final double PWM_CENTER = 1.5;
    public static final double PWM_DEADBAND_MIN = 0.55;
    public static final double PWM_MIN = 0.5;
  }

  public static final class Revolver {
    public static final double FORWARDS_SPEED = 0.075;
    public static final double BACKWARDS_SPEED = -0.075;

    public static final double VELOCITY_THRESHOLD = 5; // Dummy value
    public static final int STALL_REPS_THRESHOLD = 4;

    public static final double UNJAM_TIME = 1; // Time to unjam in seconds
  }

  public static final class Shooter {
    public static final double MAX_PERCENT_OUTPUT = 0.9;
    public static final double MAX_VOLTAGE_RAMP_RATE = 0.2;

    public static final double VELOCITY_P = 0.0;
    public static final double VELOCITY_I = 0.0;
    public static final double VELOCITY_D = 0.0;
    public static final double VELOCITY_FF = 1d / 6000d;
    public static final double VELOCITY_TOLERANCE = 10.0; // RPM

    public static final double IDLE_SPEED = 1500; // RPM
    public static final double MAX_SPEED = 6000; // RPM

    public static final double RATIO = 2d / 1d;
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
    public static final double HOME_ANGLE = 0.0; // Centre

    public static final double POT_SCALE = 3600;

    public static final double RATIO = 18d / 265d;
  }

  public static final class Vision {
    public static final double TARGET_HEIGHT = 60.0; // Inches from ground

    public static final double LIMELIGHT_HEIGHT = 39.5; // Inches from ground
    public static final double LIMELIGHT_PITCH = 0.0; // Degrees from horizontal
    public static final double LIMELIGHT_POSITION_OFFSET = 3.75; // Inches forward from centre of turret

    public static final double LIMELIGHT_HORIZONTAL_FOV = 53.0;
  }
}
