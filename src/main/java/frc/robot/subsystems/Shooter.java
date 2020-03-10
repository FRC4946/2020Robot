/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.util.Utilities;

public class Shooter extends PIDSubsystem {

  private final TalonFX m_left, m_right;
  private boolean m_key = false;

  public Shooter() {
    super(new PIDController(Constants.Shooter.VELOCITY_P, Constants.Shooter.VELOCITY_I, Constants.Shooter.VELOCITY_D));

    getController().setTolerance(Constants.Shooter.VELOCITY_TOLERANCE);

    setSetpoint(0.0);

    m_left = new TalonFX(RobotMap.CAN.TALONFX_SHOOTER_LEFT);
    m_right = new TalonFX(RobotMap.CAN.TALONFX_SHOOTER_RIGHT);
    m_right.setInverted(true);
    m_left.setInverted(false);

    m_right.configClosedloopRamp(Constants.Shooter.MAX_VOLTAGE_RAMP_RATE);
    m_left.configClosedloopRamp(Constants.Shooter.MAX_VOLTAGE_RAMP_RATE);
    m_right.configOpenloopRamp(Constants.Shooter.MAX_VOLTAGE_RAMP_RATE);
    m_left.configOpenloopRamp(Constants.Shooter.MAX_VOLTAGE_RAMP_RATE);

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

  @Override
  public void setSetpoint(double setpoint) {
    if (getController() != null) {
      getController().setSetpoint(Utilities.clip(setpoint, -Constants.Shooter.MAX_SPEED, Constants.Shooter.MAX_SPEED));
    }
    super.setSetpoint(Utilities.clip(setpoint, -Constants.Shooter.MAX_SPEED, Constants.Shooter.MAX_SPEED));
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
    m_left.set(ControlMode.PercentOutput, speed);
    m_right.set(ControlMode.PercentOutput, speed);
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

  public void setKey(boolean key) {
    m_key = key;
  }

  public boolean getKey() {
    return m_key;
  }

  /**
   * Gets the speed that the right motor is running at
   */
  public double getRightSpeed() {
    return (m_right.getSelectedSensorVelocity() / 2048d * 10d * 60d) * Constants.Shooter.RATIO;
  }

  /**
   * Gets the speed that the left motor is running at
   */
  public double getLeftSpeed() {
    return (m_left.getSelectedSensorVelocity() / 2048d * 10d * 60d) * Constants.Shooter.RATIO;
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
    super.periodic();
    SmartDashboard.putNumber("shooter/speed", getAverageSpeed());
    SmartDashboard.putNumber("shooter/setpoint", getSetpoint());
    SmartDashboard.putNumber("shooter/percentSetpoint", getAverageSpeed() / getSetpoint());
  }
}
