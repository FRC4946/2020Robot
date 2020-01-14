/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  CANSparkMax m_leftShooterMotor, m_rightShooterMotor;

  double m_setpoint = 0;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    m_leftShooterMotor = new CANSparkMax(RobotMap.CAN.SHOOTER_LEFT_MOTOR, MotorType.kBrushless);
    m_rightShooterMotor = new CANSparkMax(RobotMap.CAN.SHOOTER_RIGHT_MOTOR, MotorType.kBrushless);
    m_rightShooterMotor.setInverted(true);
    m_leftShooterMotor.setOpenLoopRampRate(Constants.SHOOTER_VOLTAGE_RAMP_RATE);
    m_rightShooterMotor.setOpenLoopRampRate(Constants.SHOOTER_VOLTAGE_RAMP_RATE);

    m_leftShooterMotor.burnFlash();
    m_rightShooterMotor.burnFlash();
  }

  public void set(double speed) {
    m_leftShooterMotor.set(speed);
    m_rightShooterMotor.set(speed);
  }

  public void stop() {
    m_leftShooterMotor.set(0);
    m_rightShooterMotor.set(0);
  }

  public double getAverageSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2.0;
  }

  public double getRightSpeed() {
    return m_rightShooterMotor.getEncoder().getVelocity();
  }

  public double getLeftSpeed() {
    return m_leftShooterMotor.getEncoder().getVelocity();
  }

  public double getSetpoint() {
    return m_setpoint;
  }

  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
  }
}
