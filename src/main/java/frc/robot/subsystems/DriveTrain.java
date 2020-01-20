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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.util.Utilities;

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
    m_leftFront = new CANSparkMax(RobotMap.CAN.DRIVE_LEFT_FRONT_SPARKMAX, MotorType.kBrushless);
    m_leftBack = new CANSparkMax(RobotMap.CAN.DRIVE_LEFT_BACK_SPARKMAX, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(RobotMap.CAN.DRIVE_RIGHT_FRONT_SPARKMAX, MotorType.kBrushless);
    m_rightBack = new CANSparkMax(RobotMap.CAN.DRIVE_RIGHT_BACK_SPARKMAX, MotorType.kBrushless);
    m_leftSide = new SpeedControllerGroup(m_leftFront, m_leftBack);
    m_rightSide = new SpeedControllerGroup(m_rightFront, m_rightBack);
    m_drive = new DifferentialDrive(m_rightSide, m_leftSide);

    m_rightSide.setInverted(true);

    m_leftEncoder = new Encoder(RobotMap.DIO.DRIVE_LEFT_ENCODER_A, RobotMap.DIO.DRIVE_LEFT_ENCODER_B);
    m_rightEncoder = new Encoder(RobotMap.DIO.DRIVE_RIGHT_ENCODER_A, RobotMap.DIO.DRIVE_RIGHT_ENCODER_B);
    m_leftEncoder.setDistancePerPulse(Constants.ENCODER_INCHES_PER_TICK);
    m_rightEncoder.setDistancePerPulse(Constants.ENCODER_INCHES_PER_TICK);

    try {
      m_gyro = new AHRS(SPI.Port.kMXP);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  //self explanitory, tank drive with 2 seperate inputs for left and right side
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  //self explanitory, arcade drive with double for speed and double for turning 1 to -1
  public void arcadeDrive(double drive, double turn) {
    m_drive.arcadeDrive(drive, turn);
  }

  //returns distance covered as a double in inches
  public double getLeftDistance() {
    return m_leftEncoder.getDistance();
  }

  //returns distance covered as a double in inches
  public double getRightDistance() {
    return m_rightEncoder.getDistance();
  }

  //returns the average distance covered from both sides as a double in inches
  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  //resets measured covered distances to zero inches
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  //Gets the current measured angle
  public double getGyroAngle() {
    try {
      return Utilities.conformAngle(m_gyro.getAngle());
    } catch (NullPointerException e) {
      System.out.println("Gyro Not Found");
      return 0;
    }
  }

}