/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Hopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  private CANSparkMax m_drumMotor;
  private CANSparkMax m_feedWheelMotor;
  private int m_repsAboveDrum = 0;
  private int m_repsAboveFeed = 0;

  public Hopper() {
    m_drumMotor = new CANSparkMax(RobotMap.CAN.DRUM_MOTOR_SPARKMAX, MotorType.kBrushless);
    m_feedWheelMotor = new CANSparkMax(RobotMap.CAN.FEED_WHEEL_MOTOR_SPARKMAX, MotorType.kBrushless);
    setDefaultCommand(new RunCommand(() -> {}, this));
  }

  /** Sets the speed of the hopper rotation
   * 
   * @param drumSpeed controls how fast the hopper spins
   * @param feedWheelSpeed controls how fast the feed wheel spins
   */
  public void setAll(double drumSpeed, double feedWheelSpeed) {
    m_drumMotor.set(drumSpeed);
    m_feedWheelMotor.set(feedWheelSpeed);

    if (Robot.m_pdp.getCurrent(0) > Constants.SHOOTER_DRUM_BURNOUT_THRESHOLD){
      incReps();
    }
    
    else {
      resetReps();
    }

    if (m_repsAbove > 4) {
      resetReps();
      new TimedHopper(-drumSpeed, -feedWheelSpeed, RobotContainer.m_hopper, timeSpentBackwards)
    }
  }

  public void incReps() {
    m_repsAboveDrum++;
  }

  public void resetReps() {
    m_repsAboveDrum = 0;
  }

  /** Stops the hopper and the feed wheel 
   * 
   */
  public void stop() {
    setAll(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}