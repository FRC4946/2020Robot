/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.revolver.UnjamRevolver;

public class Revolver extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  private CANSparkMax m_drumMotor;
  private CANSparkMax m_feedWheelMotor;
  private int m_repsAboveDrum = 0;
  private int m_repsAboveFeed = 0;
  private PowerDistributionPanel m_pdp;

  public Revolver(PowerDistributionPanel pdp) {
    m_drumMotor = new CANSparkMax(RobotMap.CAN.DRUM_MOTOR_SPARKMAX, MotorType.kBrushless);
    m_feedWheelMotor = new CANSparkMax(RobotMap.CAN.FEED_WHEEL_MOTOR_SPARKMAX, MotorType.kBrushless);
    m_pdp = pdp;
  }

  /**
   * Resets the drum reps
   */
  private void resetDrumReps() {
    m_repsAboveDrum = 0;
  }

  /**
   * Resets the feed reps
   */
  private void resetFeedReps() {
    m_repsAboveFeed = 0;
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
    if (m_pdp.getCurrent(RobotMap.PDP.DRUM_PORT) > Constants.REVOLVER_DRUM_CURRENT_THRESHOLD) {
      m_repsAboveDrum++;
    } else {
      resetDrumReps();
    }
  }

  /**
   * Sets the speed of the feed wheel
   * @param speed the speed to run the feed wheel at as a percentage from -1 to 1
   */
  public void setFeedWheel(double speed) {
    m_feedWheelMotor.set(speed);
    if (m_pdp.getCurrent(RobotMap.PDP.FEEDWHEEL_PORT) > Constants.REVOLVER_FEEDWHEEL_CURRENT_THRESHOLD) {
      m_repsAboveFeed++;
    } else {
      resetFeedReps();
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
    if (m_repsAboveFeed > Constants.REVOLVER_REPS_THRESHOLD || m_repsAboveDrum > Constants.REVOLVER_REPS_THRESHOLD) {
      new UnjamRevolver(this).schedule(false);
    }
  }
}