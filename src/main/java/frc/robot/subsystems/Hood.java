/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.util.Utilities;

public class Hood extends PIDSubsystem {

  private final Servo m_servo;
  private final AnalogInput m_pot;

  private double m_minRawAngle;

  /**
   * Creates a new Hood.
   */
  public Hood() {
    super(new PIDController(Constants.Hood.POSITION_P, Constants.Hood.POSITION_I, Constants.Hood.POSITION_D));

    m_servo = new Servo(RobotMap.PWM.HOOD_SERVO);
    m_pot = new AnalogInput(RobotMap.AIO.HOOD_POT);

    m_servo.setBounds(Constants.Hood.PWM_MAX, Constants.Hood.PWM_DEADBAND_MAX, Constants.Hood.PWM_CENTER,
        Constants.Hood.PWM_DEADBAND_MIN, Constants.Hood.PWM_MIN);

    getController().setTolerance(Constants.Hood.POSITION_TOLERANCE);

    enable();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    set(output);
  }

  @Override
  public double getMeasurement() {
    return getAngle();
  }

  /**
   * @return true if the hood is at the desired angle
   */
  public boolean atSetpoint() {
    return getController().atSetpoint();
  }

  /**
   * @return the angle of the hood in degrees
   */
  public double getAngle() {
    return ((getRawAngle() - m_minRawAngle) / Constants.Hood.POT_DEGREES_PER_HOOD_MOVE)
        * (Constants.Hood.MAX_ANGLE - Constants.Hood.MIN_ANGLE) + Constants.Hood.MIN_ANGLE;
  }

  /**
   * Gets the angle of the pot shaft
   * 
   * @return the angle of the pot shaft in degrees
   */
  private double getRawAngle() {
    return (m_pot.getVoltage() / Constants.AIO_MAX_VOLTAGE) * Constants.Hood.POT_SCALE;
  }

  /**
   * Sets the setpoint for the hood to the specified value
   *
   * @param setpoint the desired angle for the hood in degrees
   */
  public void setSetpoint(double setpoint) {
    getController().setSetpoint(Utilities.clip(setpoint, Constants.Hood.MIN_ANGLE, Constants.Hood.MAX_ANGLE));
  }

  /**
   * Gets the setpoint for the hood
   *
   * @return the setpoint
   */
  public double getSetpoint() {
    return getController().getSetpoint();
  }

  /**
   * Sets the servo to a specified speed
   *
   * @param speed the speed to set the servo to as a percent from -1 to 1
   */
  public void set(double speed) {
    if (((getRawAngle() < Constants.Hood.MIN_RAW_ANGLE || getAngle() < Constants.Hood.MIN_ANGLE) && speed < 0)
        || ((getRawAngle() > Constants.Hood.MAX_RAW_ANGLE || getAngle() > Constants.Hood.MAX_ANGLE) && speed > 0)) {
      stop();
    } else {
      m_servo.setSpeed(speed);
    }
  }

  /**
   * Resets the bottom hood angle to the current raw angle from the pot
   */
  public void resetPot() {
    m_minRawAngle = getRawAngle();
  }

  /**
   * Sets the hood speed to 0
   */
  public void stop() {
    m_servo.setSpeed(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("hood/angle", getAngle());
    SmartDashboard.putNumber("hood/setpoint", getSetpoint());
    //Not relevant until #67 is merged
    //SmartDashboard.putNumber("hood/potOffset", m_bottomValue);
  }
}
