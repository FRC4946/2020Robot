/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {

  private final TalonFX m_leftClimberMotor, m_rightClimberMotor;
  private final DoubleSolenoid m_climberSolenoid;

  public Climber() {
    m_leftClimberMotor = new TalonFX(RobotMap.CAN.TALONFX_CLIMBER_LEFT);
    m_rightClimberMotor = new TalonFX(RobotMap.CAN.TALONFX_CLIMBER_RIGHT);
    m_climberSolenoid = new DoubleSolenoid(RobotMap.PCM.CLIMBER_A, RobotMap.PCM.CLIMBER_B);

    m_rightClimberMotor.setInverted(false);
    m_leftClimberMotor.setInverted(false);
  }

  /**
   * Makes the motors run at the desired speed
   *
   * @param speed the speed that the motors will run at
   */
  public void set(double speed) {
    m_leftClimberMotor.set(ControlMode.PercentOutput, speed);
    m_rightClimberMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stops both motors
   */
  public void stop() {
    m_leftClimberMotor.set(ControlMode.PercentOutput, 0.0);
    m_rightClimberMotor.set(ControlMode.PercentOutput, 0.0);
  }

  /**
   * Sets the climber pistons to the specified position
   *
   * @param out true if forwards, false if reverse
   */
  public void setPiston(boolean out) {
    setPiston(out ? Value.kForward : Value.kReverse);
  }

  /**
   * Sets the climber pistons to the specified position
   *
   * @param value the potition to set them to
   */
  public void setPiston(Value value) {
    m_climberSolenoid.set(value);
  }

  /**
   * Toggles piston in or out
   */
  public void togglePiston() {
    setPiston(m_climberSolenoid.get() == Value.kReverse);
  }
}
