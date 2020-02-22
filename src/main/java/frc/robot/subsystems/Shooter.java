/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.util.Utilities;

public class Shooter extends PIDSubsystem {

  private final CANSparkMax m_left, m_right;

  public Shooter() {
    super(new PIDController(Constants.Shooter.VELOCITY_P, Constants.Shooter.VELOCITY_I, Constants.Shooter.VELOCITY_D));

    getController().setTolerance(Constants.Shooter.VELOCITY_TOLERANCE);

    setSetpoint(0.0);

    m_left = new CANSparkMax(RobotMap.CAN.SPARKMAX_SHOOTER_LEFT, MotorType.kBrushless);
    m_right = new CANSparkMax(RobotMap.CAN.SPARKMAX_SHOOTER_RIGHT, MotorType.kBrushless);
    m_right.setInverted(true);
    m_left.setInverted(false);
    m_left.setOpenLoopRampRate(Constants.Shooter.MAX_VOLTAGE_RAMP_RATE);
    m_right.setOpenLoopRampRate(Constants.Shooter.MAX_VOLTAGE_RAMP_RATE);

    m_left.burnFlash();
    m_right.burnFlash();

    enable();
  }

  /**
   * Gets whether the shooter is within tolerance of the desired speed
   *
   * @return true if the shooter wheel is within 100rpm of its desired speed
   */
  public boolean atSetpoint() {
    return getController().atSetpoint();
  }

  public void setSetpoint(double setpoint) {
    getController().setSetpoint(Math.min(Constants.Shooter.MAX_SPEED, Math.abs(setpoint)) * (setpoint < 0 ? -1 : 1));
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
    return m_right.getEncoder().getVelocity() * Constants.Shooter.RATIO;
  }

  /**
   * Gets the speed that the left motor is running at
   */
  public double getLeftSpeed() {
    return m_left.getEncoder().getVelocity() * Constants.Shooter.RATIO;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    output += Constants.Shooter.VELOCITY_FF * setpoint;
    set(Utilities.clip(output, -Constants.Shooter.MAX_PERCENT_OUTPUT, Constants.Shooter.MAX_PERCENT_OUTPUT));
  }

  @Override
  public double getMeasurement() {
    return getAverageSpeed();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter/speed", getAverageSpeed());
    SmartDashboard.putNumber("shooter/setpoint", getSetpoint());
    SmartDashboard.putNumber("shooter/percentSetpoint", getAverageSpeed()/getSetpoint());
  }
}
