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
import frc.robot.util.Utilities;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends PIDSubsystem {

  private CANSparkMax m_left, m_right;

  public Shooter() {
    super(new PIDController(Constants.SHOOTER_VELOCITY_CONTROL_P, Constants.SHOOTER_VELOCITY_CONTROL_I,
        Constants.SHOOTER_VELOCITY_CONTROL_D));

    getController().setTolerance(Constants.SHOOTER_SPEED_TOLERANCE);

    setSetpoint(0.0);

    m_left = new CANSparkMax(RobotMap.CAN.SHOOTER_LEFT_SPARKMAX, MotorType.kBrushless);
    m_right = new CANSparkMax(RobotMap.CAN.SHOOTER_RIGHT_SPARKMAX, MotorType.kBrushless);
    m_right.setInverted(true);
    m_left.setInverted(false);
    m_left.setOpenLoopRampRate(Constants.SHOOTER_VOLTAGE_RAMP_RATE);
    m_right.setOpenLoopRampRate(Constants.SHOOTER_VOLTAGE_RAMP_RATE);

    m_left.burnFlash();
    m_right.burnFlash();

    enable();
  }

  /**
   * Gets whether the shooter is within tolerance of the desired speed
   * @return true if the shooter wheel is within 100rpm of its desired speed
   */
  public boolean atSetpoint() {
    return getController().atSetpoint();
  }

  public void setSetpoint(double setpoint) {
    getController().setSetpoint(Math.min(Constants.SHOOTER_MAX_SPEED, Math.abs(setpoint)) * (setpoint < 0 ? -1 : 1));
  }

  public double getSetpoint() {
    return getController().getSetpoint();
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
  public void stop() {
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

  @Override
  public void useOutput(double output, double setpoint) {
    output += Constants.SHOOTER_VELOCITY_CONTROL_FF * setpoint;
    set(Utilities.clip(output, -Constants.SHOOTER_MAX_PERCENT, Constants.SHOOTER_MAX_PERCENT));
  }

  @Override
  public double getMeasurement() {
    return getAverageSpeed();
  }
}
