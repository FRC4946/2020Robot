/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.util.Utilities;

/**
 * DriveTrain Subsystem
 *
 * Subsystem for moving and tracking the movement of the robot
 */
public class DriveTrain extends SubsystemBase {
  private final SpeedControllerGroup m_leftSide, m_rightSide;
  private final CANSparkMax m_leftFront, m_leftBack, m_rightFront, m_rightBack;
  private final Encoder m_leftEncoder, m_rightEncoder;

  private final Solenoid m_highGear;

  private final AHRS m_gyro;

  private final DifferentialDrive m_drive;
  private final DifferentialDriveKinematics m_kinematics;

  private final DifferentialDriveOdometry m_odometry;

  public DriveTrain() {
    m_leftFront = new CANSparkMax(RobotMap.CAN.DRIVE_LEFT_FRONT_SPARKMAX, MotorType.kBrushless);
    m_leftBack = new CANSparkMax(RobotMap.CAN.DRIVE_LEFT_BACK_SPARKMAX, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(RobotMap.CAN.DRIVE_RIGHT_FRONT_SPARKMAX, MotorType.kBrushless);
    m_rightBack = new CANSparkMax(RobotMap.CAN.DRIVE_RIGHT_BACK_SPARKMAX, MotorType.kBrushless);

    m_highGear = new Solenoid(RobotMap.PCM.DRIVE_SHIFTER);

    m_rightFront.setInverted(true);
    m_rightBack.setInverted(true);
    m_leftFront.setInverted(false);
    m_leftBack.setInverted(false);

    m_rightFront.burnFlash();
    m_rightBack.burnFlash();
    m_leftFront.burnFlash();
    m_leftBack.burnFlash();

    m_leftSide = new SpeedControllerGroup(m_leftFront, m_leftBack);
    m_rightSide = new SpeedControllerGroup(m_rightFront, m_rightBack);
    m_drive = new DifferentialDrive(m_rightSide, m_leftSide);

    m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.DriveTrain.TRACK_WIDTH));

    m_leftEncoder = new Encoder(RobotMap.DIO.DRIVE_LEFT_ENCODER_A, RobotMap.DIO.DRIVE_LEFT_ENCODER_B);
    m_rightEncoder = new Encoder(RobotMap.DIO.DRIVE_RIGHT_ENCODER_A, RobotMap.DIO.DRIVE_RIGHT_ENCODER_B);

    m_leftEncoder.setDistancePerPulse(Constants.DriveTrain.ENCODER_INCHES_PER_TICK);
    m_rightEncoder.setDistancePerPulse(Constants.DriveTrain.ENCODER_INCHES_PER_TICK);

    AHRS gyro;
    try {
      gyro = new AHRS(SPI.Port.kMXP);
    } catch (Exception e) {
      gyro = null;
      e.printStackTrace();
    }
    m_gyro = gyro;

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-getGyroAngle()),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

    resetDriveTrain();
  }

  /**
   * Runs motors on each side at the desired speed
   *
   * @param leftSpeed  the speed that the motors on the left side will run at
   * @param rightSpeed the speed that the motors on the right side will run at
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Stops the robot
   */
  public void stop() {
    m_drive.tankDrive(0.0, 0.0);
  }

  /**
   * @param drive the forward movement of the robot
   * @param turn  the angle that the robot would turn to
   */
  public void arcadeDrive(double drive, double turn) {
    m_drive.arcadeDrive(drive, turn);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
    m_drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  /**
   * @return the left encoder's output
   */
  public double getLeftDistance() {
    return m_leftEncoder.getDistance();
  }

  /**
   * @return the right encoder's output
   */
  public double getRightDistance() {
    return m_rightEncoder.getDistance();
  }

  /**
   * @return the avrage of the left and right encoder output
   */
  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  /**
   * Resets the odometry and sets the robot position to (0, 0) with an angle of 0.
   * Also resets encoders and gyro
   */
  public void resetDriveTrain() {
    resetDriveTrain(0.0, 0.0, 0.0);
  }

  /**
   * Resets the odometry and sets the robot to the specified position. Also resets
   * encoders and gyro
   *
   * @param xPos  the x position of the robot in inches
   * @param yPos  the y position of the robot in inches
   * @param angle the angle of the robot in degrees CCW from forward
   */
  public void resetDriveTrain(double xPos, double yPos, double angle) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_gyro.reset();
    m_odometry.resetPosition(new Pose2d(xPos, yPos, new Rotation2d(xPos, yPos)), Rotation2d.fromDegrees(angle));
  }

  /**
   * @return the gyro's angle
   */
  public double getGyroAngle() {
    try {
      return Utilities.conformAngle(m_gyro.getAngle());
    } catch (NullPointerException e) {
      System.out.println("Gyro Not Found");
      return 0;
    }
  }

  public void setHighGear(boolean on) {
    m_highGear.set(on);
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(-getGyroAngle()), getLeftDistance(), getRightDistance());
  }
}
