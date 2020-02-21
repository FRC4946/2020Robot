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

public class FeedWheel extends SubsystemBase {

  private final CANSparkMax m_feedWheel;

  /**
   * Creates a new FeedWheel.
   */
  public FeedWheel() {
    m_feedWheel = new CANSparkMax(RobotMap.CAN.FEED_WHEEL_MOTOR_SPARKMAX, MotorType.kBrushless);
  }

  /**
   * Sets the feed wheel's applied voltage (open-loop).
   *
   * @param speed the voltage to apply to the motor as a percentage from -1 to 1
   */
  public void set(double speed) {
    m_feedWheel.set(speed);
  }

  /**
   * Stops the feed wheel
   */
  public void stop() {
    set(0.0);
  }
}
