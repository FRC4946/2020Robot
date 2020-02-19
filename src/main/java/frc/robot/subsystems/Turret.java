/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Turret extends PIDSubsystem {
  /**
   * Creates a new Turret.
   */

  private final TalonSRX m_turretMotor;
  private final AnalogInput m_pot;

  public Turret() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.PID_TURRET_P, Constants.PID_TURRET_I, Constants.PID_TURRET_D)
    );

    m_turretMotor = new TalonSRX(RobotMap.CAN.TURRET_TALONSRX);
    m_pot = new AnalogInput(RobotMap.AIO.TURRET_POT);
  }

  /**
   * Moves the Turret
   * @param speed the speed at which to set the turret motor to
   */

  public void set(double speed){
    m_turretMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stops the turret
   */
  public void stop(){
    m_turretMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public double getAngle() {
    return (m_pot.getAverageVoltage() / Constants.AIO_MAX_VOLTAGE) * Constants.TURRET_POT_SCALE_VALUE * Constants.TURRET_RATIO;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    setSetpoint(setpoint);
    set(output);
  }

  public double getSetpoint() {
    return getSetpoint();
  }
}
