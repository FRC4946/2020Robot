/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Utilities;

/**
 * DriveTrain Subsystem
 * <p>
 * Subsystem for moving and tracking the movement of the robot
 * </p>
 */
public class DriveTrain extends SubsystemBase {
  private SpeedControllerGroup m_leftSide, m_rightSide;
  private CANSparkMax m_leftFront, m_leftBack, m_rightFront, m_rightBack;
  private Encoder m_leftEncoder, m_rightEncoder;

  private AHRS m_gyro;

  private DifferentialDrive m_drive;

  public DriveTrain() {
    m_leftFront = new CANSparkMax(RobotMap.DRIVE_LEFT_FRONT_CANSPARKMAX, MotorType.kBrushless);
    m_leftBack = new CANSparkMax(RobotMap.DRIVE_LEFT_BACK_CANSPARKMAX, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(RobotMap.DRIVE_RIGHT_FRONT_CANSPARKMAX, MotorType.kBrushless);
    m_rightBack = new CANSparkMax(RobotMap.DRIVE_RIGHT_BACK_CANSPARKMAX, MotorType.kBrushless);
    m_leftSide = new SpeedControllerGroup(m_leftFront, m_leftBack);
    m_rightSide = new SpeedControllerGroup(m_rightFront, m_rightBack);
    m_drive = new DifferentialDrive(m_rightSide, m_leftSide);

    m_rightSide.setInverted(true);

    m_leftEncoder = new Encoder(RobotMap.DRIVE_LEFT_ENCODER_A, RobotMap.DRIVE_LEFT_ENCODER_B);
    m_rightEncoder = new Encoder(RobotMap.DRIVE_RIGHT_ENCODER_A, RobotMap.DRIVE_RIGHT_ENCODER_B);
    m_leftEncoder.setDistancePerPulse(Constants.ENCODER_INCHES_PER_TICK);
    m_rightEncoder.setDistancePerPulse(Constants.ENCODER_INCHES_PER_TICK);

    //TODO : Add error handling to gyro, or else remove try catch entirely

    try {
      m_gyro = new AHRS(SPI.Port.kMXP);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double drive, double turn) {
    m_drive.arcadeDrive(drive, turn);
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

  public double getGyroAngle() {
    try {
      return Utilities.conformAngle(m_gyro.getAngle());
    } catch (Exception e) {
      return 0;
    }
  }

}