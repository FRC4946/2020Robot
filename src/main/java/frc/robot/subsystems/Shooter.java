/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_left, m_right;
  private Servo m_hoodServo;
  private AnalogInput m_pot;
  private PIDController m_speedController, m_angleController;
  private boolean m_speedEnabled = false;
  private boolean m_angleEnabled = false;

  /**
   * Creates a new PIDShooter.
   */
  public Shooter() {
    m_speedController = new PIDController(Constants.SHOOTER_VELOCITY_CONTROL_P, Constants.SHOOTER_VELOCITY_CONTROL_I,
        Constants.SHOOTER_VELOCITY_CONTROL_D);
    m_angleController = new PIDController(Constants.SHOOTER_HOOD_P, Constants.SHOOTER_HOOD_I, Constants.SHOOTER_HOOD_D);

    m_angleController.setTolerance(Constants.HOOD_ANGLE_TOLERANCE);
    m_speedController.setTolerance(Constants.SHOOTER_SPEED_TOLERANCE);

    setAngleSetpoint(Constants.HOOD_MIN_ANGLE);
    setSpeedSetpoint(0.0);

    m_speedEnabled = true;
    m_angleEnabled = true;

    m_left = new CANSparkMax(RobotMap.CAN.SHOOTER_LEFT_SPARKMAX, MotorType.kBrushless);
    m_right = new CANSparkMax(RobotMap.CAN.SHOOTER_RIGHT_SPARKMAX, MotorType.kBrushless);
    m_right.setInverted(true);
    m_left.setInverted(false);
    m_left.setOpenLoopRampRate(Constants.SHOOTER_VOLTAGE_RAMP_RATE);
    m_right.setOpenLoopRampRate(Constants.SHOOTER_VOLTAGE_RAMP_RATE);

    m_left.burnFlash();
    m_right.burnFlash();

    m_hoodServo = new Servo(RobotMap.PWM.HOOD_SERVO);
    m_pot = new AnalogInput(RobotMap.AIO.HOOD_POT);

    m_rightHood.setBounds(2.5, 2.45, 1.5, 0.55, 0.5);
    m_leftHood.setBounds(2.5, 2.45, 1.5, 0.55, 0.5);
  }

  @Override
  public void periodic() {
    System.out.println(getAverageSpeed());
    if (m_speedEnabled) {
      useSpeedOutput(m_speedController.calculate(getAverageSpeed()));
    }
    else
      stopShooter();

    if (m_angleEnabled && !m_angleController.atSetpoint())
      useAngleSetpoint(m_angleController.calculate(getHoodAngle()));
    else
      setHoodSpeed(0.0);
  }

  public void useSpeedOutput(double output) {
    output += Constants.SHOOTER_VELOCITY_CONTROL_FF * m_speedController.getSetpoint();
    output = (output < 0 ? -1 : 1) * Math.min(Math.abs(output), 1.0);
    set(Constants.SHOOTER_MAX_PERCENT * (output));
  }

  public void useAngleSetpoint(double output) {
    setHoodSpeed(output);
  }

  public boolean atSpeedSetpoint() {
    return m_speedController.atSetpoint();
  }

  public boolean atAngleSetpoint() {
    return m_angleController.atSetpoint();
  }

  public void setSpeedSetpoint(double setpoint) {
    m_speedController.setSetpoint(Math.min(Constants.SHOOTER_MAX_SPEED, Math.abs(setpoint)) * (setpoint < 0 ? -1 : 1));
  }

  public double getSpeedSetpoint() {
    return m_speedController.getSetpoint();
  }

  public void setAngleSetpoint(double setpoint) {
    m_angleController.setSetpoint(Math.max(Constants.HOOD_MIN_ANGLE, Math.min(setpoint, Constants.HOOD_MAX_ANGLE)));
  }

  public double getAngleSetpoint() {
    return m_angleController.getSetpoint();
  }

  /**
   * Sets the motors to the desired speed
   * 
   * @param speed The speed that the motor runs at
   */
  public void set(double speed) {
    m_left.set(speed);
    m_right.set(speed);
  }

  /**
   * Stops the motors
   */
  public void stopShooter() {
    set(0.0);
  }

  /**
   * Gets the average speed of the motors
   */
  public double getAverageSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2.0;
  }

  /**
   * Gets the speed that the right motor is running at
   */
  public double getRightSpeed() {
    return m_right.getEncoder().getVelocity() * Constants.SHOOTER_RATIO;
  }

  /**
   * Gets the speed that the left motor is running at
   */
  public double getLeftSpeed() {
    return m_left.getEncoder().getVelocity() * Constants.SHOOTER_RATIO;
  }

  public void setHoodSpeed(double speed) {
    m_hoodServo.setSpeed(speed);
  }

  public double getHoodAngle() {
    return (((m_pot.getVoltage() / Constants.AIO_MAX_VOLTAGE) * Constants.HOOD_POT_SCALE_VALUE)
        - Constants.HOOD_POT_OFFSET_VALUE) * (Constants.HOOD_MAX_ANGLE - Constants.HOOD_MIN_ANGLE)
        + Constants.HOOD_MIN_ANGLE;
  }

  public void setEnabled(boolean enabled) {
    setEnabledHood(enabled);
    setEnabledShooter(enabled);
  }

  public void setEnabledShooter(boolean enabled) {
    m_speedEnabled = enabled;
  }

  public void setEnabledHood(boolean enabled) {
    m_angleEnabled = enabled;
  }
}
