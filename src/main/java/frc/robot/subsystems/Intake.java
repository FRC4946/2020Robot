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
  private DoubleSolenoid m_frontSolenoid, m_backSolenoid;
  private VictorSPX m_intake;

  public Intake() {
    m_frontSolenoid = new DoubleSolenoid(RobotMap.PCM.FRONT_INTAKE_A, RobotMap.PCM.FRONT_INTAKE_B);
    m_backSolenoid = new DoubleSolenoid(RobotMap.PCM.BACK_INTAKE_A, RobotMap.PCM.BACK_INTAKE_B);

    m_intake = new VictorSPX(RobotMap.CAN.INTAKE_VICTORSPX);
  }

  public void set(double speed) {
    m_intake.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    set(0.0);
  }

  public void setFrontExtended(boolean extended) {
    m_frontSolenoid.set(extended ? Value.kForward : Value.kReverse);
  }

  public void setFrontExtended(Value value) {
    m_frontSolenoid.set(value);
  }

  public void setBackExtended(boolean extended) {
    m_backSolenoid.set(extended ? Value.kForward : Value.kReverse);
  }

  public void setBackExtended(Value value) {
    m_backSolenoid.set(value);
  }

  public void setExtended(boolean extended) {
    setFrontExtended(extended);
    setBackExtended(extended);
  }
}
