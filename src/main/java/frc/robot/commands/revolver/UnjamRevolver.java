/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.revolver;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Revolver;

public class UnjamRevolver extends CommandBase {

  private final Revolver m_revolver;
  private final Timer m_timer;

  /**
   * Creates a new UnjamRevolver command.
   */
  public UnjamRevolver(Revolver revolver) {
    m_revolver = revolver;
    m_timer = new Timer();
    addRequirements(m_revolver);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    m_revolver.set(Constants.Revolver.BACKWARDS_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_revolver.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > Constants.Revolver.UNJAM_TIME;
  }
}
