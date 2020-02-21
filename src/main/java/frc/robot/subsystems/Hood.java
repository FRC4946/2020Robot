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
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.util.Utilities;

public class Hood extends PIDSubsystem {

  private Servo m_servo;
  private AnalogInput m_pot;

  /**
   * Creates a new Hood.
   */
  public Hood() {
    super(new PIDController(Constants.SHOOTER_HOOD_P, Constants.SHOOTER_HOOD_I, Constants.SHOOTER_HOOD_D));

    m_servo = new Servo(RobotMap.PWM.HOOD_SERVO);
    m_pot = new AnalogInput(RobotMap.AIO.HOOD_POT);

    m_servo.setBounds(Constants.HOOD_PWM_MAX, Constants.HOOD_PWM_DEADBAND_MAX, Constants.HOOD_PWM_CENTER,
        Constants.HOOD_PWM_DEADBAND_MIN, Constants.HOOD_PWM_MIN);

    getController().setTolerance(Constants.HOOD_ANGLE_TOLERANCE);
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
   * Gets the angle the hood is at in degrees
   */
  public double getAngle() {
    return (((m_pot.getVoltage() / Constants.AIO_MAX_VOLTAGE) * Constants.HOOD_POT_SCALE_VALUE)
        - Constants.HOOD_POT_OFFSET_VALUE) * (Constants.HOOD_MAX_ANGLE - Constants.HOOD_MIN_ANGLE)
        + Constants.HOOD_MIN_ANGLE;
  }

  /**
   * Sets the setpoint for the hood to the specified value
   * @param setpoint the desired angle for the hood in degrees
   */
  public void setSetpoint(double setpoint) {
    getController.setSetpoint(Utilities.clip(setpoint, Constants.HOOD_MIN_ANGLE, Constants.HOOD_MAX_ANGLE));
  }

  /**
   * Gets the setpoint for the hood
   * @return the setpoint
   */
  public double getSetpoint() {
    return getController().getSetpoint();
  }

  /**
   * Sets the servo to a specified speed
   * @param speed the speed to set the servo to as a percent from -1 to 1
   */
  public void set(double speed) {
    if ((getAngle() < Constants.HOOD_MIN_ANGLE && speed < 0) || (getAngle() > Constants.HOOD_MAX_ANGLE && speed > 0)) {
      stop();
    } else {
      m_servo.setSpeed(speed);
    }
  }

  /**
   * Sets the hood speed to 0
   */
  public void stop() {
    m_servo.setSpeed(0.0);
  }
}
