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

  // TODO : Finalize Values

  // #region OI

  // JOYSTICKS

  /**
   * OI Port Numbers For Joystick Axes
   */
  public static final class JOYSTICK_AXIS {

    // DRIVER

    public static final int DRIVE = 0;
    public static final int TURN = 1;

    public static final int CLIMB_1 = 4;
    public static final int CLIMB_2 = 5;

    public static final int INTAKE = 2;
    public static final int OUTTAKE = 3;

    // OPERATOR

    public static final int TURRET = 0;
  }

  /**
   * OI Port Numbers For Joystick Buttons
   */
  public static final class JOYSTICK_BUTTON {

    // DRIVER

    public static final int CLIMB = 1; // A on X Box Controller
    public static final int DRIVER_SHOOT = 3;

    public static final int INTAKE = 5;

    // OPERATOR

    public static final int OPERATOR_SHOOT = 3;

  }

  /**
   * OI Port Numbers For Joysticks
   */
  public static final class JOYSTICK {
    public static final int DRIVE_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;
  }

  // #endregion

  // #region IO

  // MOTORS AND SENSORS

  /**
   * CAN IDs
   */
  public static final class CAN {
    public static final int CLIMBER_LEFT_SPARKMAX = 10;
    public static final int CLIMBER_RIGHT_SPARKMAX = 9;

    public static final int SHOOTER_LEFT_SPARKMAX = 6;
    public static final int SHOOTER_RIGHT_SPARKMAX = 5;

    public static final int DRIVE_RIGHT_FRONT_SPARKMAX = 1;
    public static final int DRIVE_RIGHT_BACK_SPARKMAX = 2;
    public static final int DRIVE_LEFT_FRONT_SPARKMAX = 3;
    public static final int DRIVE_LEFT_BACK_SPARKMAX = 4;

    public static final int TURRET_TALONSRX = 0;

    public static final int DRUM_MOTOR_SPARKMAX = 7;
    public static final int FEED_WHEEL_MOTOR_SPARKMAX = 8;

    public static final int INTAKE_VICTORSPX = 0;

    public static final int CONTROL_PANEL_TALONSRX = 11;

    public static final int PDP = 0;
  }

  /**
   * PCM Port Numbers
   */
  public static final class PCM {
    public static final int INTAKE_A = 0;
    public static final int INTAKE_B = 1;

    public static final int DRIVE_SHIFTER = 4;

    public static final int CLIMBER_A = 5;
    public static final int CLIMBER_B = 6;
  }

  /**
   * PDP Port Numbers
   */
  public static final class PDP {
    public static final int DRUM_PORT = 0;
    public static final int FEEDWHEEL_PORT = 1;
  }

  /**
   * DIO Port Numbers
   */
  public static final class DIO {
    public static final int DRIVE_LEFT_ENCODER_A = 0;
    public static final int DRIVE_LEFT_ENCODER_B = 1;

    public static final int DRIVE_RIGHT_ENCODER_A = 2;
    public static final int DRIVE_RIGHT_ENCODER_B = 3;

    public static final int WHEEL_ENCODER_A = 4;
    public static final int WHEEL_ENCODER_B = 5;
  }

  /**
   * Analog Port Numbers
   */
  public static final class AIO {
    public static final int HOOD_POT = 1;
  }

  /**
   * PWM Port Numbers
   */
  public static final class PWM {
    public static final int HOOD_SERVO = 0;
  }

  // #endregion
}
