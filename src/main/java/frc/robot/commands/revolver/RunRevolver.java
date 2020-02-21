/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.revolver;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Revolver;

public class RunRevolver extends CommandBase {

  private double m_revolverSpeed;
  private Revolver m_revolver;

  /**
   * Creates a new RunRevolver command.
   */
  public RunRevolver(double revolverSpeed, Revolver revolver) {
    m_revolver = revolver;
    m_revolverSpeed = revolverSpeed;
    addRequirements(m_revolver);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_revolver.set(m_revolverSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_revolver.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
