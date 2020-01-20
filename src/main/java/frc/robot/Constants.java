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

    // TODO : Fill In Inches Per Tick
    public static final double ENCODER_INCHES_PER_TICK = 1;

    public static final double SHOOTER_VOLTAGE_RAMP_RATE = 0.2;

    public static final double SHOOTER_VELOCITY_CONTROL_P = 0.0;
    public static final double SHOOTER_VELOCITY_CONTROL_I = 0.0;
    public static final double SHOOTER_VELOCITY_CONTROL_D = 0.0;

    // TODO : Fill In Climber Pot Max Distance and other variables
    public static final double CLIMBER_POT_MAX_DISTANCE = 1;

    public static final double CLIMBER_UP_HEIGHT = 0;
    public static final double CLIMBER_DOWN_HEIGHT = 0;

    public static final double MIN_CLIMBER_SPEED = 0.1;
    public static final double MAX_CLIMBER_SPEED = 0.1;

    public static final double DIO_MAX_VOLTAGE = 5.0;

    public static final double PID_CLIMBER_P = 0;
    public static final double PID_CLIMBER_I = 0;
    public static final double PID_CLIMBER_D = 0;
    
    public static final double DEFAULT_DEADZONE = 0.1;

    public static final double ROBOT_START_X = 0.0;
    public static final double ROBOT_START_Y = 0.0;
    public static final double ROBOT_START_ANGLE = 0.0;

    // TODO : Fill in the variables for kinematics
    public static double TRACK_WIDTH = 0.0
    
    public static double LINEAR_VELOCITY = 0.0
    public static double ANGULAR_VELOCITY = 0.0

    public static double LEFT_METER_PER_SECOND = 0.0
    public static double RIGHT_METER_PER_SECOND = 0.0
}
