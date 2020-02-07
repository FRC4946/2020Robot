/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Hopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  private CANSparkMax m_hopperMotor;

  public Hopper() {
    m_hopperMotor = new CANSparkMax(RobotMap.CAN.HOPPER_MOTOR_SPARKMAX, MotorType.kBrushless);
  }

  public void set(double speed) {
    m_hopperMotor.set(speed);
  }

  public void stop() {
    m_hopperMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
