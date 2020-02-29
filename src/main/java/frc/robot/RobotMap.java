/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Contains all port numbers for motors, sensors, joysticks & OI stuff, etc...
 *
 * Only reason these values aren't in constants is because I like having the
 * port numbers and OI stuff separate from the PID Tunings, etc...
 */
public final class RobotMap {

  // #region OI

  /**
   * OI Port Numbers For Joystick Axes
   */
  public static final class JOYSTICK_AXIS {

    // DRIVER

    public static final int DRIVE = 1;
    public static final int TURN = 0;

    public static final int CLIMB_1 = 4;
    public static final int CLIMB_2 = 5;

    public static final int INTAKE = 2;
    public static final int OUTTAKE = 3;

    // OPERATOR

    public static final int TURRET = 0;
    public static final int HOOD = 1;
  }

  /**
   * OI Port Numbers For Joystick Buttons
   */
  public static final class JOYSTICK_BUTTON {

    // DRIVER

    public static final int CLIMB = 8; // Start
    public static final int DRIVER_SHOOT = 3; // X
    public static final int EMERGENCY_SHOOT = 4; // Y
    public static final int MANUAL_UNJAM = 1; // A

    public static final int INTAKE = 6; // RB

    public static final int SHIFT_GEAR = 2; // B

    public static final int REVOLVER = 5; // LB

    // OPERATOR

    public static final int USE_LIMELIGHT = 3; // X
    public static final int OPERATOR_SPIN_UP = 2; // B

    public static final int EXTEND_CONTROL_PANEL = 6; // RB

    public static final int PRESET_1 = 4; // Y

    public static final int MANUAL_MODE = 5; // LB

    public static final int HOOD_BUTTON = 8; // Start
  }

  /**
   * OI Port Numbers For Joysticks
   */
  public static final class JOYSTICK {
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;
  }

  // #endregion

  // #region IO

  /**
   * CAN IDs
   */
  public static final class CAN {
    public static final int SPARKMAX_DRIVE_RIGHT_FRONT = 1;
    public static final int SPARKMAX_DRIVE_RIGHT_BACK = 2;
    public static final int SPARKMAX_DRIVE_LEFT_FRONT = 3;
    public static final int SPARKMAX_DRIVE_LEFT_BACK = 4;
    public static final int SPARKMAX_REVOLVER = 7;
    public static final int SPARKMAX_FEED_WHEEL = 8;

    public static final int TALONFX_CLIMBER_LEFT = 2;
    public static final int TALONFX_CLIMBER_RIGHT = 3;
    public static final int TALONFX_SHOOTER_RIGHT = 4;
    public static final int TALONFX_SHOOTER_LEFT = 5;

    public static final int TALONSRX_TURRET = 1;
    public static final int TALONSRX_CONTROL_PANEL = 0;

    public static final int VICTORSPX_INTAKE = 0;
  }

  /**
   * PCM Port Numbers
   */
  public static final class PCM {
    public static final int INTAKE_A = 0;
    public static final int INTAKE_B = 1;

    public static final int DRIVE_SHIFTER = 4;

    public static final int CLIMBER_A = 2;
    public static final int CLIMBER_B = 3;

    public static final int CONTROL_PANEL = 5;
  }

  /**
   * DIO Port Numbers
   */
  public static final class DIO {
    public static final int DRIVE_LEFT_ENCODER_A = 0;
    public static final int DRIVE_LEFT_ENCODER_B = 1;

    public static final int DRIVE_RIGHT_ENCODER_A = 2;
    public static final int DRIVE_RIGHT_ENCODER_B = 3;
  }

  /**
   * Analog Port Numbers
   */
  public static final class AIO {
    public static final int HOOD_POT = 0;
  }

  /**
   * PWM Port Numbers
   */
  public static final class PWM {
    public static final int HOOD_SERVO = 0;
  }

  // #endregion
}
