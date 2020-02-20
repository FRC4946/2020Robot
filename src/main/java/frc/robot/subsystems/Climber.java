/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {

  private CANSparkMax m_leftClimberMotor, m_rightClimberMotor;
  private CANEncoder m_leftEncoder, m_rightEncoder;
  private DoubleSolenoid m_climberSolenoid;


  public Climber() {
    m_leftClimberMotor = new CANSparkMax(RobotMap.CAN.CLIMBER_LEFT_SPARKMAX, MotorType.kBrushless);
    m_rightClimberMotor = new CANSparkMax(RobotMap.CAN.CLIMBER_RIGHT_SPARKMAX, MotorType.kBrushless);
    m_climberSolenoid = new DoubleSolenoid(RobotMap.PCM.CLIMBER_A, RobotMap.PCM.CLIMBER_B);

    m_leftEncoder = m_leftClimberMotor.getEncoder();
    m_rightEncoder = m_rightClimberMotor.getEncoder();

    setPiston(false);
    m_rightClimberMotor.setInverted(true);
    m_leftClimberMotor.setInverted(false);
    m_rightClimberMotor.burnFlash();
    m_leftClimberMotor.burnFlash();
  }

  /**
   * Makes the motors run at the desired speed
   *
   * @param speed the speed that the motors will run at
   */
  public void set(double speed) {
    m_leftClimberMotor.set(speed);
    m_rightClimberMotor.set(speed);
  }

  /**
   * Stops both motors
   */
  public void stop() {
    m_leftClimberMotor.set(0.0);
    m_rightClimberMotor.set(0.0);
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

  /**
   * @return how far extended the left climber is
   */
  public double getLeftDistance() {
    return m_leftEncoder.getPosition();
  }

  /**
   * @return how far extended the right climber is
   */
  public double getRightDistance() {
    return m_rightEncoder.getPosition();
  }

  /**
   * @return the avrage of the left and right encoder output
   */
  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  /**
   * resets the encoder
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
}
