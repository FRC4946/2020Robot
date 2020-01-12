/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Contains all port numbers for motors, sensors, joysticks & OI stuff, etc...
 * <p>
 * Only reason these values aren't in constants is because I like having the port numbers and OI stuff
 * separate from the PID Tunings, etc...
 * </p>
 * @author Jacob 
 */
public final class RobotMap {

    //TODO : Finalize Values

    //JOYSTICKS

    public static final int DRIVE_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;

    public static final int DRIVE_AXIS = 0;
    public static final int TURN_AXIS = 1;

    //MOTORS AND SENSORS

    public static final int DRIVE_LEFT_ENCODER_A = 0;
    public static final int DRIVE_LEFT_ENCODER_B = 1;

    public static final int DRIVE_RIGHT_ENCODER_A = 2;
    public static final int DRIVE_RIGHT_ENCODER_B = 3;

    public static final int CLIMBER_LEFT_MOTOR = 0;
    public static final int CLIMBER_RIGHT_MOTOR = 1;  
}
