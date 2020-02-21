/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

  private final DoubleSolenoid m_solenoid;
  private final VictorSPX m_intake;

  public Intake() {
    m_solenoid = new DoubleSolenoid(RobotMap.PCM.INTAKE_A, RobotMap.PCM.INTAKE_B);
    m_intake = new VictorSPX(RobotMap.CAN.VICTORSPX_INTAKE);
  }

  public void set(double speed) {
    m_intake.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    set(0.0);
  }

  public void setExtended(boolean extended) {
    m_solenoid.set(extended ? Value.kForward : Value.kReverse);
  }

  public void setExtended(Value value) {
    m_solenoid.set(value);
  }
}
