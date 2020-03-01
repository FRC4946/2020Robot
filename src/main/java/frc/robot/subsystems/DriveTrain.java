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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final CANSparkMax m_leftFront, m_leftBack, m_rightFront, m_rightBack;

  private final Encoder m_leftEncoder, m_rightEncoder;

  private final Solenoid m_highGear;

  private final AHRS m_gyro;

  private final DifferentialDriveKinematics m_kinematics;

  private final DifferentialDriveOdometry m_odometry;

  private final PIDController m_leftController, m_rightController;

  public DriveTrain() {
    m_leftFront = new CANSparkMax(RobotMap.CAN.SPARKMAX_DRIVE_LEFT_FRONT, MotorType.kBrushless);
    m_leftBack = new CANSparkMax(RobotMap.CAN.SPARKMAX_DRIVE_LEFT_BACK, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(RobotMap.CAN.SPARKMAX_DRIVE_RIGHT_FRONT, MotorType.kBrushless);
    m_rightBack = new CANSparkMax(RobotMap.CAN.SPARKMAX_DRIVE_RIGHT_BACK, MotorType.kBrushless);

    m_highGear = new Solenoid(RobotMap.PCM.DRIVE_SHIFTER);

    m_leftEncoder = new Encoder(RobotMap.DIO.DRIVE_LEFT_ENCODER_A, RobotMap.DIO.DRIVE_LEFT_ENCODER_B);
    m_rightEncoder = new Encoder(RobotMap.DIO.DRIVE_RIGHT_ENCODER_A, RobotMap.DIO.DRIVE_RIGHT_ENCODER_B);

    m_leftEncoder.setReverseDirection(true);
    m_rightEncoder.setReverseDirection(false);
    m_leftEncoder.setDistancePerPulse(Constants.DriveTrain.ENCODER_METERS_PER_TICK);
    m_rightEncoder.setDistancePerPulse(Constants.DriveTrain.ENCODER_METERS_PER_TICK);

    m_leftFront.follow(m_leftBack);
    m_rightFront.follow(m_rightBack);

    /*
     * Not sure why these all need to be inverted, but they do
     */
    m_rightFront.setInverted(false);
    m_rightBack.setInverted(false);
    m_leftFront.setInverted(true);
    m_leftBack.setInverted(true);

    m_rightFront.burnFlash();
    m_rightBack.burnFlash();
    m_leftFront.burnFlash();
    m_leftBack.burnFlash();

    m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.DriveTrain.TRACK_WIDTH));

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

    m_leftController = new PIDController(Constants.DriveTrain.VELOCITY_P, Constants.DriveTrain.VELOCITY_I,
        Constants.DriveTrain.VELOCITY_D);
    m_rightController = new PIDController(Constants.DriveTrain.VELOCITY_P, Constants.DriveTrain.VELOCITY_I,
        Constants.DriveTrain.VELOCITY_D);

    resetDriveTrain();
  }

  /**
   * Stops the robot
   */
  public void stop() {
    m_leftBack.set(0.0);
    m_rightBack.set(0.0);
  }

  /**
   * @param drive the forward movement of the robot
   * @param turn  the angle that the robot would turn to
   */
  public void arcadeDrive(double drive, double turn) {

    drive = Utilities.clip(drive, -1, 1);
    turn = Utilities.clip(turn, -1, 1);

    drive = Math.copySign(Math.pow(drive, 2), drive);
    turn = Math.copySign(Math.pow(turn, 2), turn);

    m_leftBack.set(drive - turn);
    m_rightBack.set(drive + turn);
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
   * @return the average of the left and right encoder output
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
   * Resets the PID Controllers
   */
  public void resetControllers() {
    m_rightController.reset();
    m_leftController.reset();
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
    resetControllers();
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_gyro.reset();
    m_odometry.resetPosition(new Pose2d(xPos, yPos, new Rotation2d(xPos, yPos)), Rotation2d.fromDegrees(angle));
  }

  /**
   * Gets the speed of the right side of the drivetrain
   *
   * @return the speed of the right side of the drivetrain in meters/second
   */
  public double getRightVelocity() {
    return m_rightEncoder.getRate();
  }

  /**
   * Gets the speed of the left side of the drivetrain
   *
   * @return the speed of the left side of the drivetrain in meters/second
   */
  public double getLeftVelocity() {
    return m_leftEncoder.getRate();
  }

  /**
   * Sets the velocity for the left side of the drivetrain, must be called once
   * every scheduler cycle
   *
   * @param velocity the velocity to set the left side at in meters per second
   */
  public void setLeftVelocity(double velocity) {
    double output = m_leftController.calculate(getLeftVelocity(), velocity);
    output += Constants.DriveTrain.VELOCITY_FF * velocity;

    m_leftBack.set(output);
  }

  /**
   * Sets the velocity for the right side of the drivetrain, must be called once
   * every scheduler cycle
   *
   * @param velocity the velocity to set the right side at in meters per second
   */
  public void setRightVelocity(double velocity) {
    double output = m_rightController.calculate(getRightVelocity(), velocity);
    output += Constants.DriveTrain.VELOCITY_FF * velocity;

    m_rightBack.set(output);
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

  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setHighGear(boolean on) {
    m_highGear.set(on);
  }

  public boolean isHighGear() {
    return m_highGear.get();
  }

  @Override
  public void periodic() {
    // m_odometry.update(Rotation2d.fromDegrees(-getGyroAngle()), getLeftDistance(), getRightDistance());

    // SmartDashboard.putNumberArray("drive/robotPosition",
    //     new double[] { getPose().getTranslation().getX(), getPose().getTranslation().getY() });
    // SmartDashboard.putNumber("drive/angle", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("drive/encoders/leftEncoder", getLeftDistance());
    SmartDashboard.putNumber("drive/encoders/rightEncoder", getRightDistance());
    SmartDashboard.putNumber("drive/gyroAngle", getGyroAngle());
    SmartDashboard.putBoolean("drive/lowGear", m_highGear.get());
    SmartDashboard.putNumber("drive/speed", Math.abs((getLeftVelocity() + getRightVelocity()) / 2d));
  }
}
