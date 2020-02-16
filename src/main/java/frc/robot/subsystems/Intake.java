/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  private Solenoid m_frontSolenoid, m_backSolenoid;
  private TalonSRX m_back, m_front;

  public Intake() {
    m_frontSolenoid = new Solenoid(RobotMap.PCM.FRONT_INTAKE);
    m_backSolenoid = new Solenoid(RobotMap.PCM.BACK_INTAKE);

    m_back = new TalonSRX(RobotMap.CAN.BACK_INTAKE_TALONSRX);
    m_front = new TalonSRX(RobotMap.CAN.FRONT_INTAKE_TALONSRX);
  }

  public void setFront(double speed) {
    m_front.set(ControlMode.PercentOutput, speed);
  }

  public void setBack(double speed) {
    m_back.set(ControlMode.PercentOutput, speed);
  }

  public void set(double speed) {
    setFront(speed);
    setBack(speed);
  }

  public void stop() {
    set(0.0);
  }

  public void setFrontExtended(boolean extended) {
    m_frontSolenoid.set(extended);
  }

  public void setBackExtended(boolean extended) {
    m_backSolenoid.set(extended);
  }

  public void setExtended(boolean extended) {
    setFrontExtended(extended);
    setBackExtended(extended);
  }
}
