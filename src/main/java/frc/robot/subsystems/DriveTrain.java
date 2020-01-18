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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
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
  
  private DifferentialDriveKinematic m_kinematic;
  private var m_chassisSpeeds;
  private DifferentialDriveWheelSpeeds m_wheelSpeeds;

  private DifferentialDriveOdometry m_odometry;

  public DriveTrain() {
    m_leftFront = new CANSparkMax(RobotMap.CAN.DRIVE_LEFT_FRONT_SPARKMAX, MotorType.kBrushless);
    m_leftBack = new CANSparkMax(RobotMap.CAN.DRIVE_LEFT_BACK_SPARKMAX, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(RobotMap.CAN.DRIVE_RIGHT_FRONT_SPARKMAX, MotorType.kBrushless);
    m_rightBack = new CANSparkMax(RobotMap.CAN.DRIVE_RIGHT_BACK_SPARKMAX, MotorType.kBrushless);

    m_rightFront.setInverted(true);
    m_rightBack.setInverted(true);

    m_rightFront.burnFlash();
    m_rightBack.burnFlash();
    m_leftFront.burnFlash();
    m_leftBack.burnFlash();

    m_leftSide = new SpeedControllerGroup(m_leftFront, m_leftBack);
    m_rightSide = new SpeedControllerGroup(m_rightFront, m_rightBack);
    m_drive = new DifferentialDrive(m_rightSide, m_leftSide);
    
    m_kinematic = new DifferentialDriveKinematic = new (Units.inchesToMeters(Constants.TRACK_WIDTH));
    m_chassisSpeeds = new ChassisSpeeds(Constants.LINEAR_VELOCITY, 0, Constants.ANGULAR_VELOCITY);
    m_wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);

    m_leftEncoder = new Encoder(RobotMap.DIO.DRIVE_LEFT_ENCODER_A, RobotMap.DIO.DRIVE_LEFT_ENCODER_B);
    m_rightEncoder = new Encoder(RobotMap.DIO.DRIVE_RIGHT_ENCODER_A, RobotMap.DIO.DRIVE_RIGHT_ENCODER_B);
    m_leftEncoder.setDistancePerPulse(Constants.ENCODER_INCHES_PER_TICK);
    m_rightEncoder.setDistancePerPulse(Constants.ENCODER_INCHES_PER_TICK);

    try {
      m_gyro = new AHRS(SPI.Port.kMXP);
    } catch (Exception e) {
      e.printStackTrace();
    }

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-getGyroAngle()), new Pose2d(
        Constants.ROBOT_START_X, Constants.ROBOT_START_Y, Rotation2d.fromDegrees(Constants.ROBOT_START_ANGLE)));
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double drive, double turn) {
    m_drive.arcadeDrive(drive, turn);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn){
    m_drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  public void stop(){
    m_drive.tankDrive(0.0, 0.0);
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

  public double getLeftVelocity(double leftVelocity){
    leftVelocity = wheelSpeeds.leftMetersPerSecond;
  }

  public double getRightVelocity(double rightVelocity){
    leftVelocity = wheelSpeeds.rightMetersPerSecond;
  }



  /**
   * Resets the odometry and sets the robot to the inputted position
   * 
   * @param xPos  the x position of the robot in inches
   * @param yPos  the y position of the robot in inches
   * @param angle the angle of the robot in degrees (WPILIB Format, degrees
   *              counterclockwise with 0 being straight ahead)
   */
  public void resetOdometry(double xPos, double yPos, double angle) {
    resetEncoders();
    m_odometry.resetPosition(new Pose2d(xPos, yPos, new Rotation2d(xPos, yPos)), Rotation2d.fromDegrees(angle));
  }

  /**
   * Resets the odometry and sets the robot to the inputted position
   */
  public void resetOdometry() {
    resetOdometry(0, 0, 0);
  }

  public void resetGyro() {
    m_gyro.reset();
    resetOdometry();
  }

  public double getGyroAngle() {
    try {
      return Utilities.conformAngle(m_gyro.getAngle());
    } catch (NullPointerException e) {
      System.out.println("Gyro Not Found");
      return 0;
    }
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(-getGyroAngle()), getLeftDistance(), getRightDistance());
  }

}