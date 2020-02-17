/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {

  private TalonSRX m_leftClimberMotor, m_rightClimberMotor;
  private AnalogInput m_pot;
  private DoubleSolenoid m_climberSolenoid;


  public Climber() {
    m_leftClimberMotor = new TalonSRX(RobotMap.CAN.CLIMBER_LEFT_TALONSRX);
    m_rightClimberMotor = new TalonSRX(RobotMap.CAN.CLIMBER_RIGHT_TALONSRX);
    m_climberSolenoid = new DoubleSolenoid(RobotMap.PCM.CLIMBER_A, RobotMap.PCM.CLIMBER_B);
    m_pot = new AnalogInput(RobotMap.AIO.CLIMBER_POT);

    setPiston(false);
  }

  /**
   * Makes the motors run at the desired speed
   * @param speed the speed that the motors will run at
   */
  public void set(double speed) {
    m_leftClimberMotor.set(ControlMode.PercentOutput, speed);
    m_rightClimberMotor.set(ControlMode.PercentOutput, -speed);
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
   * @param out true if forwards, false if reverse
   */
  public void setPiston(boolean out) {
    setPiston(out ? Value.kForward : Value.kReverse);
  }

  /**
   * Sets the climber pistons to the specified position
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
   * @return how far extended the climber is
   */
  public double getDistance() {
    return (m_pot.getAverageVoltage() / Constants.AIO_MAX_VOLTAGE) * Constants.CLIMBER_POT_SCALE_VALUE;
  }
}