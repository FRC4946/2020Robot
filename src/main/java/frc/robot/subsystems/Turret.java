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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {

  private final TalonSRX m_turretMotor;

  public Turret() {
    m_turretMotor = new TalonSRX(RobotMap.CAN.TURRET_TALONSRX);
    m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.TURRET_PID_LOOP_INDEX, Constants.TURRET_PID_TIMEOUT);
    m_turretMotor.configPotentiometerTurns(Constants.TURRET_POT_TURNS);
  }
  /**
   * Moves the Turret
   *
   * @param speed the speed at which to set the turret motor to
   */
  public void set(double speed) {
    m_turretMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stops the turret
   */
  public void stop() {
    set(0.0);
  }

  public double getAngle() {
    return (m_turretMotor.getSensorCollection().getAnalogInRaw() / Constants.TURRET_TALONPOT_SCALE_VALUE)*360;
    //return (m_pot.getAverageVoltage() / Constants.AIO_MAX_VOLTAGE) * Constants.TURRET_POT_SCALE_VALUE * Constants.TURRET_RATIO;
  }

  public void setSetpoint(double setpoint){
    m_turretMotor.getSensorCollection().setAnalogPosition(setpoint, Constants.TURRET_PID_TIMEOUT);
  }
}
