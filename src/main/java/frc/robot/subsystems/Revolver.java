/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.revolver.UnjamRevolver;

public class Revolver extends SubsystemBase {

  private final CANSparkMax m_revolver;
  private int m_drumReps = 0;

  private final UnjamRevolver m_unjam;

  private final Timer m_timer;

  public Revolver() {
    m_revolver = new CANSparkMax(RobotMap.CAN.SPARKMAX_REVOLVER, MotorType.kBrushless);
    m_revolver.setOpenLoopRampRate(Constants.Revolver.MAX_VOLTAGE_RAMP_RATE);
    m_revolver.setClosedLoopRampRate(Constants.Revolver.MAX_VOLTAGE_RAMP_RATE);
    m_revolver.burnFlash();
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
    m_revolver.set(speed);
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

  @Override
  public void periodic() {
    if (m_revolver.get() > 0.0 && m_revolver.getEncoder().getVelocity() < Constants.Revolver.VELOCITY_THRESHOLD) {
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
