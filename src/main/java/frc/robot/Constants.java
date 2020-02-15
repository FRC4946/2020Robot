/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double AIO_MAX_VOLTAGE = 5.0;

    public static final double DEFAULT_DEADZONE = 0.1;
    
    public static final double ENCODER_INCHES_PER_TICK = 1;

    public static final double SHOOTER_VOLTAGE_RAMP_RATE = 0.2;

    //#region PID Tunings

    public static final double SHOOTER_VELOCITY_CONTROL_P = 0.0;
    public static final double SHOOTER_VELOCITY_CONTROL_I = 0.0;
    public static final double SHOOTER_VELOCITY_CONTROL_D = 0.0;
    public static final double SHOOTER_VELOCITY_CONTROL_FF = 0.0;

    public static final double SHOOTER_HOOD_P = 0.0;
    public static final double SHOOTER_HOOD_I = 0.0;
    public static final double SHOOTER_HOOD_D = 0.0;

    public static final double PID_CLIMBER_P = 0;
    public static final double PID_CLIMBER_I = 0;
    public static final double PID_CLIMBER_D = 0;

    public static final double PID_TURRET_P = 0.014;
    public static final double PID_TURRET_I = 0.0105;
    public static final double PID_TURRET_D = 0.0002;
    public static final double PID_TURRET_FF = 0.06;

    //#endregion

    //#region Shooter Details

    public static final double SHOOTER_MAX_PERCENT = 0.9;

    public static final double SHOOTER_MAX_SPEED = 6000;

    public static final double SHOOTER_RATIO = 2d/1d;

    public static final double HOOD_ANGLE_TOLERANCE = 2;
    
    public static final double HOOD_POT_SCALE_VALUE = 5;
    public static final double HOOD_POT_OFFSET_VALUE = 1;

    public static final double HOOD_MIN_ANGLE = 27.4;
    public static final double HOOD_MAX_ANGLE = 82.5;
    
    //#endregion

    //#region Limelight Details

    public static final double TARGET_HEIGHT = 60.0;
    public static final double LIMELIGHT_HEIGHT = 39.5;

    public static final double[] LIMELIGHT_OFFSET_ZERO_ROTATION = new double[] {0.0, 3.75};

    public static final double[] TURRET_OFFSET = new double[] {0.0, 0.0};

    public static final double LIMELIGHT_ANGLE_OFFSET = 0.0;
    public static final double LIMELIGHT_PITCH = 0.0;

    public static final double LIMELIGHT_HORIZONTAL_FOV = 53.0;

    //#endregion

    //#region Turret Details

    public static final double TURRET_MAX_PERCENT = 0.9;

    public static final double TURRET_PID_TOLERANCE = 0.3;

    public static final double TURRET_ROTATION_MIN = 0;

    public static final double TURRET_ROTATION_MAX = 180;

    public static final double TURRET_RATIO = 18d/32d;

    public static final double TURRET_VELOCITY_ERROR_THRESHOLD = 0.0;

    public static final double TURRET_POT_SCALE_VALUE = 3600;

    public static final double TURRET_HOME_ANGLE = 90;

    //#endregion

    //#region Climber Details

    public static final double CLIMBER_POT_SCALE_VALUE = 10.0;

    public static final double CLIMBER_POT_MAX_DELTA = 1;
    public static final double CLIMBER_POT_MIN = 0;

    //#endregion
}
