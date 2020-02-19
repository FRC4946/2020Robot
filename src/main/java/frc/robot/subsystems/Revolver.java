/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.revolver.UnjamRevolver;

public class Revolver extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  private final CANSparkMax m_drumMotor;
  private final CANSparkMax m_feedWheelMotor;
  private final Encoder m_drumEncoder;
  private final Encoder m_feedWheelEncoder;
  private final PowerDistributionPanel m_pdp;

  private boolean m_drumJammed;
  private boolean m_feedJammed;

  public Revolver(PowerDistributionPanel pdp) {
    m_drumMotor = new CANSparkMax(RobotMap.CAN.DRUM_MOTOR_SPARKMAX, MotorType.kBrushless);
    m_feedWheelMotor = new CANSparkMax(RobotMap.CAN.FEED_WHEEL_MOTOR_SPARKMAX, MotorType.kBrushless);
    m_pdp = pdp;
    m_drumJammed = false;
    m_feedJammed = false; 
    m_drumEncoder = new Encoder(RobotMap.DIO.REVOLVER_DRUM_ENCODER_A, RobotMap.DIO.REVOLVER_DRUM_ENCODER_B);
    m_drumEncoder.setMaxPeriod(Constants.REVOLVER_DRUM_ENCODER_MAXPERIOD);
    m_feedWheelEncoder = new Encoder(RobotMap.DIO.REVOLVER_FEEDWHEEL_ENCODER_A, RobotMap.DIO.REVOLVER_FEEDWHEEL_ENCODER_B);
    m_feedWheelEncoder.setMaxPeriod(Constants.REVOLVER_FEEDWHEEL_ENCODER_MAXPERIOD);
  }

  private void resetDrumEncoder(){
    m_drumEncoder.reset();
  }

  private void resetFeedWheelEncoder(){
    m_feedWheelEncoder.reset();
  }

  public void resetEncoders(){
    resetDrumEncoder();
    resetFeedWheelEncoder();
  }

  /**
   * Sets the speed of the revolver drum and feed wheel
   * @param drumSpeed controls how fast the drum spins as a percentage from -1 to 1
   * @param feedWheelSpeed controls how fast the feed wheel spins as a percentage from -1 to 1
   */
  public void setAll(double drumSpeed, double feedWheelSpeed) {
    setDrum(drumSpeed);
    setFeedWheel(feedWheelSpeed);
  }

  /**
   * Sets the speed of the drum
   * @param speed the speed to run the drum at as a percentage from -1 to 1
   */
  public void setDrum(double speed) {
    m_drumMotor.set(speed);
    if (m_drumEncoder.getStopped()){
      m_drumJammed = true;
    }
    else{
      m_drumJammed = false;
    }
  }

  /**
   * Sets the speed of the feed wheel
   * @param speed the speed to run the feed wheel at as a percentage from -1 to 1
   */
  public void setFeedWheel(double speed) {
    m_feedWheelMotor.set(speed);
    if (m_feedWheelEncoder.getStopped()) {
      m_feedJammed = true;
    }
    else{
      m_feedJammed = false;
    } 
  }

  /**
   * Stops the drum
   */
  public void stopDrum(){
    setDrum(0.0);
  }

  /**
   * Stops the drum
   */
  public void stopFeedWheel(){
    setFeedWheel(0.0);
  }

  /**
   * Stops the drum and the feed wheel
   */
  public void stop() {
    setAll(0, 0);
  }

  @Override
  public void periodic() {
    if (m_drumJammed || m_feedJammed) {
      new UnjamRevolver(this).schedule(false);
    }
  }
}