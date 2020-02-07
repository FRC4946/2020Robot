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

public class Hopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  private CANSparkMax m_DrumMotor;
  private CANSparkMax m_FeedwheelMotor;
  private int m_repsAboveDrum = 0;
  private int m_repsAboveFeed = 0;

  public Hopper() {
    m_DrumMotor = new CANSparkMax(RobotMap.CAN.HOPPER_MOTOR_SPARKMAX, MotorType.kBrushless);
    setDefaultCommand(new RunCommand(() -> {
      set(0.3);
      if (Robot.m_pdp.getCurrent(0) > Constants.SHOOTER_DRUM_BURNOUT_THRESHOLD)
        incReps();
      else
        resetReps();
      if (m_repsAbove > 4) {
        resetReps();
        new TimedRunMotor(-0.3, 1.0, this).schedule();
      }
    }, this));
  }

  /** Sets the speed of the hopper rotation
   * 
   * @param speed controls how fast the hopper spins
   */
  public void set(double speed) {
    m_DrumMotor.set(speed);
  }

  public void incReps() {
    m_repsAboveDrum++;
  }

  public void resetReps() {
    m_repsAboveDrum = 0;
  }

  /** Stops the hopper from spinning
   * 
   */
  public void stop() {
    m_DrumMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}