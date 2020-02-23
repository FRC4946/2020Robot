/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.util.Utilities;

public class Turret extends SubsystemBase {

  private final TalonSRX m_turretMotor;

  public Turret() {
    m_turretMotor = new TalonSRX(RobotMap.CAN.TALONSRX_TURRET);
    m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

    m_turretMotor.configNominalOutputForward(0.0);
    m_turretMotor.configNominalOutputReverse(0.0);
    m_turretMotor.configPeakOutputReverse(Constants.Turret.MAX_PERCENT_OUTPUT);
    m_turretMotor.configPeakOutputReverse(-Constants.Turret.MAX_PERCENT_OUTPUT);

    m_turretMotor.configAllowableClosedloopError(0, degreesToSensorUnits(Constants.Turret.POSITION_TOLERANCE));

    m_turretMotor.config_kP(0, Constants.Turret.POSITION_P);
    m_turretMotor.config_kI(0, Constants.Turret.POSITION_I);
    m_turretMotor.config_kD(0, Constants.Turret.POSITION_D);
    m_turretMotor.config_kF(0, 0.0);
  }

  /**
   * Sets the turret's applied voltage (open-loop).
   *
   * @param speed the voltage to apply to the motor as a percentage from -1 to 1
   */
  public void set(double speed) {
    if ((getAngle() < Constants.Turret.MIN_ANGLE && speed > 0.0)
        || (getAngle() > Constants.Turret.MAX_ANGLE && speed < 0.0)
        || (getAngle() > Constants.Turret.MIN_ANGLE && getAngle() < Constants.Turret.MAX_ANGLE)) {
      m_turretMotor.set(ControlMode.PercentOutput, speed);
    } else {
      m_turretMotor.set(ControlMode.PercentOutput, 0.0);
    }
  }

  /**
   * Sets the turret PID setpoint.
   */
  public void setSetpoint(double setpoint) {
    m_turretMotor.set(ControlMode.Position,
        degreesToSensorUnits(Utilities.clip(setpoint, Constants.Turret.MIN_ANGLE, Constants.Turret.MAX_ANGLE)) + 512);
  }

  /**
   * Stops the turret.
   */
  public void stop() {
    // TODO: Do we want to apply a 0-velocity PID to hold position?
    set(0.0);
  }

  /**
   * @return the current turret angle
   */
  public double getAngle() {
    return sensorUnitsToDegrees(m_turretMotor.getSelectedSensorPosition() - 512) - Constants.Turret.ANGLE_OFFSET;
  }

  /**
   * @return the current turret angle setpoint
   */
  public double getSetpoint() {
    if (m_turretMotor.getControlMode() == ControlMode.PercentOutput) {
      return 0.0;
    } else {
      return m_turretMotor.getClosedLoopTarget();
    }
  }

  /**
   * @return the current turret angle setpoint
   */
  public boolean atSetpoint() {
    return sensorUnitsToDegrees(m_turretMotor.getClosedLoopError()) < Constants.Turret.POSITION_TOLERANCE && Math
        .abs(sensorUnitsToDegrees(m_turretMotor.getSelectedSensorVelocity())) < Constants.Turret.VELOCITY_TOLERANCE;
  }

  /**
   * Converts raw TalonSRX sensor units to degrees.
   */
  private double sensorUnitsToDegrees(int raw) {
    return (raw / 1023d) * Constants.Turret.POT_SCALE * Constants.Turret.RATIO;
  }

  /**
   * Converts degrees to raw TalonSRX sensor units.
   */
  private int degreesToSensorUnits(double degrees) {
    return (int) Math.round(degrees / (Constants.Turret.POT_SCALE * Constants.Turret.RATIO) * 1023d);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turret/angle", getAngle());
    SmartDashboard.putNumber("turret/setpoint", getSetpoint());
  }
}
