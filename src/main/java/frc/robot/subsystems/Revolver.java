/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.revolver.UnjamRevolver;

public class Revolver extends SubsystemBase {

  private final WPI_TalonFX m_revolver;
  private int m_drumReps = 0;

  private final UnjamRevolver m_unjam;

  private final Timer m_timer;

  public Revolver() {
    m_revolver = new WPI_TalonFX(RobotMap.CAN.TALONFX_REVOLVER);
    m_revolver.configOpenloopRamp(0.05);
    m_revolver.configClosedloopRamp(0.05);
    m_unjam = new UnjamRevolver(this);
    m_timer = new Timer();
    m_timer.start();
  }

  /**
   * Sets the revolver's applied voltage (open-loop).
   *
   * @param speed the voltage to apply to the motor as a percentage from -1 to 1
   */
  public void set(double speed) {
    m_revolver.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stops the drum
   */
  public void stop() {
    set(0.0);
  }

  public void resetUnjamTimer() {
    m_timer.reset();
  }

  public double getVelocity() {
    return m_revolver.getSelectedSensorVelocity() / 2048d * 10d * 60d;
  }

  @Override
  public void periodic() {
    if (m_revolver.get() > 0.0 && getVelocity() < Constants.Revolver.VELOCITY_THRESHOLD) {
      m_drumReps++;
    } else {
      m_drumReps = 0;
    }

    if (m_drumReps > Constants.Revolver.STALL_REPS_THRESHOLD && m_timer.get() > Constants.Revolver.UNJAM_COOLDOWN) {
      m_drumReps = 0;
      m_unjam.schedule(false);
    }

    SmartDashboard.putBoolean("revolver/jammed", m_unjam.isScheduled());
  }
}
