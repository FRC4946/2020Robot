/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Turret extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  TalonSRX m_turretMotor;
  AnalogInput m_pot;

  public Turret(){
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
}
