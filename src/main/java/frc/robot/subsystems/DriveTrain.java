/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {
  private SpeedControllerGroup m_leftSide, m_rightSide;
  private Encoder m_leftEncoder, m_rightEncoder;

  public DriveTrain(){
    m_leftSide = new SpeedControllerGroup(null);
    m_rightSide = new SpeedControllerGroup(null);
    m_rightSide.setInverted(true);

    m_leftEncoder = new Encoder(RobotMap.DRIVE_LEFT_ENCODER_A,
      RobotMap.DRIVE_LEFT_ENCODER_B);
    m_rightEncoder = new Encoder(RobotMap.DRIVE_RIGHT_ENCODER_A,
      RobotMap.DRIVE_RIGHT_ENCODER_B);
    m_leftEncoder.setDistancePerPulse(Constants.ENCODER_INCHES_PER_TICK);
    m_rightEncoder.setDistancePerPulse(Constants.ENCODER_INCHES_PER_TICK);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_leftSide.set(leftSpeed);
    m_rightSide.set(rightSpeed);
  }

  public void arcadeDrive(double drive, double turn) {
    tankDrive(drive + turn, drive - turn);
  }

  public double getLeftDistance() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

}
