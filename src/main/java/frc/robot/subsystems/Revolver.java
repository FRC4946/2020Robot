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
import frc.robot.commands.revolver.UnjamRevolver;

public class Revolver extends SubsystemBase {

  private final CANSparkMax m_drumMotor;
  private final CANSparkMax m_feedWheelMotor;
  private int m_drumReps = 0;

  public Revolver() {
    m_drumMotor = new CANSparkMax(RobotMap.CAN.DRUM_MOTOR_SPARKMAX, MotorType.kBrushless);
    m_feedWheelMotor = new CANSparkMax(RobotMap.CAN.FEED_WHEEL_MOTOR_SPARKMAX, MotorType.kBrushless);
  }

  /**
   * Sets the speed of the revolver drum and feed wheel
   *
   * @param drumSpeed      controls how fast the drum spins as a percentage from
   *                       -1 to 1
   * @param feedWheelSpeed controls how fast the feed wheel spins as a percentage
   *                       from -1 to 1
   */
  public void setAll(double drumSpeed, double feedWheelSpeed) {
    setDrum(drumSpeed);
    setFeedWheel(feedWheelSpeed);
  }

  public void resetReps() {
    m_drumReps = 0;
  }

  /**
   * Sets the speed of the drum
   *
   * @param speed the speed to run the drum at as a percentage from -1 to 1
   */
  public void setDrum(double speed) {
    m_drumMotor.set(speed);
  }

  /**
   * Sets the speed of the feed wheel
   *
   * @param speed the speed to run the feed wheel at as a percentage from -1 to 1
   */
  public void setFeedWheel(double speed) {
    m_feedWheelMotor.set(speed);
  }

  /**
   * Stops the drum
   */
  public void stopDrum() {
    setDrum(0.0);
  }

  /**
   * Stops the drum
   */
  public void stopFeedWheel() {
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
    if (m_drumMotor.get() > 0.0 && m_drumMotor.getEncoder().getVelocity() < Constants.REVOLVER_VELOCITY_THRESHOLD) {
      m_drumReps++;
    } else {
      resetReps();
    }

    if (m_drumReps > Constants.REVOLVER_REPS_THRESHOLD) {
      resetReps();
      new UnjamRevolver(this).schedule(false);
    }
  }
}
