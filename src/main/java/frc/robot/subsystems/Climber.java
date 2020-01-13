/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;

public class Climber extends SubsystemBase {

  private TalonSRX m_leftClimberMotor, m_rightClimberMotor;
  /**
   * Creates a new Climber.
   */

  public Climber() {
    m_leftClimberMotor = new TalonSRX(RobotMap.CLIMBER_LEFT_MOTOR);
    m_rightClimberMotor = new TalonSRX(RobotMap.CLIMBER_RIGHT_MOTOR);
  }

  public void set(double speed) {
    m_leftClimberMotor.set(ControlMode.PercentOutput, speed);
    m_rightClimberMotor.set(ControlMode.PercentOutput,speed);
  }

  public void stop() {
    m_leftClimberMotor.set(ControlMode.PercentOutput, 0.0);
    m_rightClimberMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}