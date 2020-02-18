/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotMap;

public class PIDTurret extends PIDSubsystem {
  /**
   * Creates a new PIDTurret.
   */

  private final TalonSRX m_turretMotor;
  private final AnalogInput m_pot;

  public PIDTurret() {  
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
    m_turretMotor = new TalonSRX(RobotMap.CAN.TURRET_TALONSRX);
    m_pot = new AnalogInput(RobotMap.AIO.TURRET_POT);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
