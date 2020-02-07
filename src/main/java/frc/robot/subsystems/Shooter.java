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
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  CANSparkMax m_leftShooterMotor, m_rightShooterMotor;
  Victor m_leftHoodMotor, m_rightHoodMotor;
  AnalogInput m_leftHoodPOT, m_rightHoodPOT;

  double m_setpoint = 0;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    m_leftShooterMotor = new CANSparkMax(RobotMap.CAN.SHOOTER_LEFT_SPARKMAX, MotorType.kBrushless);
    m_rightShooterMotor = new CANSparkMax(RobotMap.CAN.SHOOTER_RIGHT_SPARKMAX, MotorType.kBrushless);
    m_rightShooterMotor.setInverted(true);
    m_leftShooterMotor.setOpenLoopRampRate(Constants.SHOOTER_VOLTAGE_RAMP_RATE);
    m_rightShooterMotor.setOpenLoopRampRate(Constants.SHOOTER_VOLTAGE_RAMP_RATE);

    m_leftShooterMotor.burnFlash();
    m_rightShooterMotor.burnFlash();

    m_leftHoodMotor = new Victor(RobotMap.CAN.LEFT_HOOD_MOTOR);
    m_rightHoodMotor = new Victor(RobotMap.CAN.RIGHT_HOOD_MOTOR);
    m_leftHoodPOT = new AnalogInput(RobotMap.AIO.LEFT_HOOD_POT);
    m_rightHoodPOT = new AnalogInput(RobotMap.AIO.RIGHT_HOOD_POT);
  }

  /**
   * Sets the motors to the desired speed
   * @param speed The speed that the motor runs at
   */
  public void setShooter(double speed) {
    m_leftShooterMotor.set(speed);
    m_rightShooterMotor.set(speed);
  }

  /**
   * Stops the motors
   */
  public void stopShooter() {
    m_leftShooterMotor.set(0);
    m_rightShooterMotor.set(0);
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
    return m_rightShooterMotor.getEncoder().getVelocity();
  }

  /**
   * Gets the speed that the left motor is running at
   */
  public double getLeftSpeed() {
    return m_leftShooterMotor.getEncoder().getVelocity();
  }

  /**
   * Get PID set point
   */
  public double getSetpoint() {
    return m_setpoint;
  }

  /**
   * Sets the PID setpoint 
   * @param setpoint desired set point
   */
  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
  }

  /** Sets the speed of the left hood motor
   * 
   * @param speed controls the speed of the motor
   */
  public void setLeftHoodMotor(double speed) {
    m_leftHoodMotor.set(speed);
  }
  
  /** Sets the speed of the right hood motor
   * 
   * @param speed controls the speed of the motor
   */
  public void setRightHoodMotor(double speed) {
    m_rightHoodMotor.set(speed);
  }
  
  /** Sets the speed of the both hood motors
   * 
   * @param speed controls the speed of both of the motor
   */
  public void setBothHoodMotors(double speed) {
    m_leftHoodMotor.set(speed);
    m_rightHoodMotor.set(speed);
  }

  /** Stops the left hood motor
   * 
   */
  public void stopLeftHoodMotor() {
    m_leftHoodMotor.set(0);
  }
  
  /** Stops the right hood motor
   * 
   */
  public void stopRightHoodMotor() {
    m_rightHoodMotor.set(0);
  }

  /** Stops both of the hood motors
   * 
   */
  public void stopBothHoodMotors() {
    m_leftHoodMotor.set(0);
    m_rightHoodMotor.set(0);
  }

  
  /** Runs the left motor until the volts read form the left POT equual the target volts
   * 
   * @param volts target volts for the POT
   * @param speed controls the speed of the motor
   */
  public void setLeftHoodMotorsVolts(double volts, double speed) {
    if (m_leftHoodPOT.getVoltage() < volts) {
      setLeftHoodMotor(speed);
    } else if (m_leftHoodPOT.getVoltage() > volts) {
      setLeftHoodMotor(-speed);
    } else {
      stopLeftHoodMotor();
    }
  }

  /** Runs the right motor until the volts read form the right POT equual the target volts
   * 
   * @param volts target volts for the POT
   * @param speed controls the speed of the motor
   */
  public void setRightHoodMotorsVolts(double volts, double speed) {
    if (m_rightHoodPOT.getVoltage() < volts) {
      setRightHoodMotor(speed);
    } else if (m_rightHoodPOT.getVoltage() > volts) {
      setRightHoodMotor(-speed);
    } else {
      stopRightHoodMotor();
    }
  }

  /** Runs the both motor until the volts read form the both POT equual the target volts
   * 
   * @param volts target volts for the POT
   * @param speed controls the speed of the motor
   */
  public void setBothHoodMotorsVolts(double volts, double speed) {
    if (m_leftHoodPOT.getVoltage() < volts) {
      setLeftHoodMotor(speed);
    } else if (m_leftHoodPOT.getVoltage() > volts) {
      setLeftHoodMotor(-speed);
    } else {
      stopLeftHoodMotor();
    }
    
    if (m_rightHoodPOT.getVoltage() < volts) {
      setRightHoodMotor(speed);
    } else if (m_rightHoodPOT.getVoltage() > volts) {
      setRightHoodMotor(-speed);
    } else {
      stopRightHoodMotor();
    }
  }
}
