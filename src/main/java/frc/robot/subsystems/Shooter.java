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
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  CANSparkMax m_leftShooterMotor, m_rightShooterMotor;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    m_leftShooterMotor = new CANSparkMax(RobotMap.SHOOTER_LEFT_MOTOR, MotorType.kBrushless);
    m_rightShooterMotor = new CANSparkMax(RobotMap.SHOOTER_RIGHT_MOTOR, MotorType.kBrushless);
    m_rightShooterMotor.setInverted(true);
  }

  public void set(double speed) {
    m_leftShooterMotor.set(speed);
    m_rightShooterMotor.set(speed);
  }

  public void stop() {
    m_leftShooterMotor.set(0);
    m_rightShooterMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
