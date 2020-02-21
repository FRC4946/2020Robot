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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {

  private final TalonSRX m_turretMotor;

  public Turret() {
    m_turretMotor = new TalonSRX(RobotMap.CAN.TURRET_TALONSRX);
    m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

    m_turretMotor.configNominalOutputForward(0.0);
    m_turretMotor.configNominalOutputReverse(0.0);
    m_turretMotor.configPeakOutputReverse(Constants.TURRET_MAX_PERCENT_OUTPUT);
    m_turretMotor.configPeakOutputReverse(-Constants.TURRET_MAX_PERCENT_OUTPUT);

		m_turretMotor.configAllowableClosedloopError(0, degreesToSensorUnits(Constants.TURRET_PID_TOLERANCE));

    m_turretMotor.config_kP(0, Constants.PID_TURRET_P);
    m_turretMotor.config_kI(0, Constants.PID_TURRET_I);
    m_turretMotor.config_kD(0, Constants.PID_TURRET_D);
    m_turretMotor.config_kF(0, 0.0);
  }

  /**
   * Sets the turret's applied voltage (open-loop).
   */
  public void set(double speed) {
    m_turretMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Sets the turret PID setpoint.
   */
  public void setSetpoint(double setpoint) {
    m_turretMotor.set(ControlMode.Position, degreesToSensorUnits(setpoint));
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
    return sensorUnitsToDegrees(m_turretMotor.getSelectedSensorPosition());
  }

  /**
   * @return the current turret angle setpoint
   */
  public double getSetpoint() {
    return m_turretMotor.getClosedLoopTarget();
  }

  /**
   * Converts raw TalonSRX sensor units to degrees.
   */
  private double sensorUnitsToDegrees(int raw) {
    return ((raw - 512) / 1023d) * Constants.TURRET_POT_SCALE_VALUE * Constants.TURRET_RATIO;
  }

  /**
   * Converts degrees to raw TalonSRX sensor units.
   */
  private int degreesToSensorUnits(double degrees) {
    return (int) Math.round(degrees / (Constants.TURRET_POT_SCALE_VALUE * Constants.TURRET_RATIO) * 1023) + 512;
  }
}
