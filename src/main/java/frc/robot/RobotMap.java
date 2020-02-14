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
 * Only reason these values aren't in constants is because I like having the
 * port numbers and OI stuff separate from the PID Tunings, etc...
 * </p>
 * 
 * @author Jacob
 */
public final class RobotMap {

    // TODO : Finalize Values

    // JOYSTICKS

    /**
     * OI Port Numbers For Joystick Axes
     */
    public static final class JOYSTICK_AXIS {
        public static final int DRIVE_AXIS = 0;
        public static final int TURN_AXIS = 1;
    }

    /**
     * OI Port Numbers For Joystick Buttons
     */
    public static final class JOYSTICK_BUTTON {
        public static final int CLIMB_BUTTON = 1; // A on X Box Controller
    }

    /**
     * OI Port Numbers For Joysticks
     */
    public static final class JOYSTICK {
        public static final int DRIVE_JOYSTICK = 0;
        public static final int OPERATOR_JOYSTICK = 1;
    }

    // #endregion

    // MOTORS AND SENSORS

    /**
     * CAN IDs
     */
    public static final class CAN {
        public static final int CLIMBER_LEFT_TALONSRX = 0;
        public static final int CLIMBER_RIGHT_TALONSRX = 1;

        public static final int SHOOTER_LEFT_SPARKMAX = 2;
        public static final int SHOOTER_RIGHT_SPARKMAX = 3;

        public static final int LEFT_CONVEYOR_BELT_TALONSRX = 4;
        public static final int RIGHT_CONVEYOR_BELT_TALONSRX = 5;
        public static final int VERTICAL_CONVEYOR_BELT_TALONSRX = 6;

        public static final int DRIVE_RIGHT_FRONT_SPARKMAX = 7;
        public static final int DRIVE_RIGHT_BACK_SPARKMAX = 8;
        public static final int DRIVE_LEFT_FRONT_SPARKMAX = 9;
        public static final int DRIVE_LEFT_BACK_SPARKMAX = 10;
    }

    /**
     * PCM Ports
     */
    public static final class PCM {
        public static final int PCM_CLIMBER_SOLENOID_A = 0;
        public static final int PCM_CLIMBER_SOLENOID_B = 1;
    }

    // #region IO

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
        public static final int CLIMBER_POT = 0;
    }

    // #endregion
}
