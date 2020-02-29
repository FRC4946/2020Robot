/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Utilities;

/**
 * Limelight Object
 */
public class Limelight extends SubsystemBase {

  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_tx;
  private final NetworkTableEntry m_ty;
  private final NetworkTableEntry m_ta;
  private final NetworkTableEntry m_tv;
  private final NetworkTableEntry m_ts;

  private double m_cached_tx;
  private double m_cached_ty;
  private double m_cached_ta;
  private double m_cached_tv;
  private double m_cached_ts;

  /**
   * Initializes all network table entries, and creates object Turns on vision and
   * LEDs by default
   */
  public Limelight() {
    m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");

    setVision(false);
    setLED(false);

    m_tx = m_networkTable.getEntry("tx");
    m_ty = m_networkTable.getEntry("ty");
    m_ta = m_networkTable.getEntry("ta");
    m_tv = m_networkTable.getEntry("tv");
    m_ts = m_networkTable.getEntry("ts");
  }

  /**
   * Returns whether or not the limelight has any valid targets
   *
   * @return true if the limelight detects a valid target
   */
  public boolean getHasTarget() {
    return (m_cached_tv == 1);
  }

  /**
   * Returns the coordinates of the centre of the targeting box
   *
   * @return An array containing the x offset and y offset of the target {xOffset,
   *         yOffset}
   */
  public double[] getOffset() {
    return new double[] { m_cached_tx, m_cached_ty };
  }

  /**
   * Returns the x-offset from the target
   *
   * @return A double containing the x-offset
   */
  public double getOffsetX() {
    return m_cached_tx;
  }

  /**
   * Returns the area of the target as a percentage of the whole image (0% to
   * 100%)
   *
   * @return the area of the target
   */
  public double getTargetArea() {
    return m_cached_ta;
  }

  /**
   * Returns the skew of the targeting box from -90 to 0 degrees
   *
   * @return the skew of the targeting box
   */
  public double getTargetSkew() {
    return m_cached_ts;
  }

  /**
   * Returns true if the LED is on and false if it isn't
   *
   * @return true if the LED is on
   */
  public boolean getLED() {
    return (!(m_networkTable.getEntry("ledMode").getDouble(0.0) == 1.0));
  }

  /**
   * Turns the leds on the limelight on or off
   *
   * @param on Should the LEDs on the limelight be on
   */
  public void setLED(boolean on) {
    if (on) {
      m_networkTable.getEntry("ledMode").setNumber(3);
    } else {
      m_networkTable.getEntry("ledMode").setNumber(1);
    }

  }

  /**
   * Sets the leds on the limelight to the pipeline default
   */
  public void setLEDDefault() {
    m_networkTable.getEntry("ledMode").setNumber(0);
  }

  /**
   * Enables or disables vision processcing on the limelight
   *
   * @param on Should vision processing be on or off
   */
  public void setVision(boolean on) {
    if (on) {
      m_networkTable.getEntry("camMode").setNumber(0);
    } else {
      m_networkTable.getEntry("camMode").setNumber(1);
    }

  }

  /**
   * Sets the limelight to the desired vision processcing pipeline
   *
   * @param pipeline the pipeline to set the limelight to from 0 to 9
   * @throws IndexOutOfBoundsException if the pipeine number is not between 0 and
   *                                   9
   */
  public void setPipeline(int pipeline) {

    if (pipeline > 9 || pipeline < 0) {
      throw new IndexOutOfBoundsException("Pipeline Index Is Out Of Bounds");
    }
    m_networkTable.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Estimates the distance to a target
   *
   * @return estimated distance in metres
   */
  public double findDistance() {

    if (!getHasTarget()) {
      return 0.0;
    }

    return (Constants.Vision.TARGET_HEIGHT - Constants.Vision.LIMELIGHT_HEIGHT)
        * Math.sin(Math.toRadians(90 - (m_cached_ty + Constants.Vision.LIMELIGHT_PITCH)))
        / Math.sin(Math.toRadians((m_cached_ty + Constants.Vision.LIMELIGHT_PITCH)));
  }

  /**
   * Finds the vector between the robot and the limelight target in a turret
   * relative coordinate system
   *
   * @return the components of the vector from the robot to the target in inches
   *         relative to the turret
   */
  public double[] findTurretRelativePosition() {

    if (!getHasTarget()) {
      return new double[] {0.0, 0.0};
    }

    double distance = findDistance();
    return new double[] { Math.cos(Math.toRadians(90 - m_cached_tx)) * findDistance(),
        Constants.Vision.LIMELIGHT_POSITION_OFFSET + Math.sin(Math.toRadians(90 - m_cached_tx)) * distance };
  }

  /**
   * Finds the vector between the robot and the limelight target in a field
   * relative coordinate system
   *
   * @param turretAngle the angle of the turret in degrees
   * @param robotAngle  the angle of the robot in degrees
   * @return the components of the vector from the robot to the target in inches
   *         relative to the field
   */
  public double[] findFieldRelativePosition(double robotAngle, double turretAngle) {

    if (!getHasTarget()) {
      return new double[] {0.0, 0.0};
    }

    double[] position = findTurretRelativePosition();
    double angle = 90 - (getAngleOffset() + turretAngle + robotAngle);
    double magnitude = Math.sqrt(Math.pow(position[0], 2) + Math.pow(position[1], 2));
    return new double[] { Math.cos(Math.toRadians(angle)) * magnitude, Math.sin(Math.toRadians(angle)) * magnitude };
  }

  /**
   * Finds the vector between the robot and the inner goal in a turret relative
   * coordinate system
   *
   * @param robotAngle  the angle of the robot in degrees
   * @param turretAngle the angle of the turret in degrees
   * @return the components of the vector from the robot to the inner goal in
   *         inches relative to the turret
   */
  public double[] findTurretRelativeInnerGoalPosition(double robotAngle, double turretAngle) {

    if (!getHasTarget()) {
      return new double[] {0.0, 0.0};
    }

    double position[] = findFieldRelativeInnerGoalPosition(robotAngle, turretAngle);
    double magnitude = Math.sqrt(Math.pow(position[0], 2) + Math.pow(position[1], 2));
    double angle = (90 - (position[0] == 0 ? 90 : Math.toDegrees(Math.atan(position[1] / position[0])))) - robotAngle
        - turretAngle;
    if (position[1] < 0) { // Quadrant 2 or 3
      angle += 180;
    }
    return new double[] { Math.cos(Math.toRadians(angle)) * magnitude, Math.sin(Math.toRadians(angle)) * magnitude };
  }

  /**
   * Finds the vector between the robot and the inner goal in a field relative
   * coordinate system
   *
   * @param robotAngle  the angle of the robot in degrees
   * @param turretAngle the of the turret in degrees
   * @return the components of the vector from the robot to the inner goal in
   *         inches relative to the field
   */
  public double[] findFieldRelativeInnerGoalPosition(double robotAngle, double turretAngle) {

    if (!getHasTarget()) {
      return new double[] {0.0, 0.0};
    }

    double[] position = findFieldRelativePosition(robotAngle, turretAngle);
    return new double[] { position[0] + position[1] + Constants.Vision.INNER_HOLE_OFFSET };
  }

  /**
   * Gets the angle between the turret and the limelight target
   *
   * @return the angle between the turret and the limelight target in degrees
   */
  public double getAngleOffset() {

    if (!getHasTarget()) {
      return 0.0;
    }

    double[] position = findTurretRelativePosition();
    double angle = 90 - (position[0] == 0 ? 90 : Math.toDegrees(Math.atan((position[1] / position[0]))));

    return ((Math.abs(angle) > Constants.Vision.LIMELIGHT_HORIZONTAL_FOV ? -(180 - angle) : angle));
  }

  /**
   * Gets the angle between the turret and the calculated position of the inner
   * goal in degrees
   *
   * @param robotAngle  the angle of the robot in degrees
   * @param turretAngle the angle of the turret in degrees
   * @return the angle between the turret and the inner goal's calculated position
   *         in degrees
   */
  public double getInnerGoalAngleOffset(double robotAngle, double turretAngle) {

    if (!getHasTarget()) {
      return 0.0;
    }

    double[] position = findTurretRelativeInnerGoalPosition(robotAngle, turretAngle);
    double angle = 90 - (position[0] == 0 ? 90 : Math.toDegrees(Math.atan((position[1] / position[0]))));

    return ((Math.abs(angle) > Constants.Vision.LIMELIGHT_HORIZONTAL_FOV ? -(180 - angle) : angle));
  }

  /**
   * Gets a turret hood angle from a distance to the limelight target
   *
   * @return the desired hood angle in degrees
   */
  public double getHoodAngle() {

    if (!getHasTarget()) {
      return Constants.Hood.MIN_ANGLE;
    }

    return Utilities.clip(0.21356 * findDistance() + 32.2392, Constants.Hood.MIN_ANGLE, 74);
  }

  /**
   * Gets a shooter wheel speed from a distance to the limelight target
   *
   * @return the desired wheel speed in rpm
   */
  public double getShooterSpeed() {
    return 4600;
  }

  @Override
  public void periodic() {
    
    m_cached_tx = m_tx.getDouble(0);
    m_cached_ty = m_ty.getDouble(0);
    m_cached_ta = m_ta.getDouble(0);
    m_cached_tv = m_tv.getDouble(0);
    m_cached_ts = m_ts.getDouble(0);

    // SmartDashboard.putNumberArray("vision/targetPosition", findTurretRelativePosition());
    // SmartDashboard.putNumber("vision/targetDistance", findDistance());
    SmartDashboard.putNumber("vision/targetAngle", getAngleOffset());
  }
}
