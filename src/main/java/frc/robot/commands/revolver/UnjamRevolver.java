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
  
  Revolver m_revolver;
  
  Timer m_timer = new Timer();

  /**
   * Creates a new UnjamRevolver.
   */
  public UnjamRevolver(Revolver revolver) {
    m_revolver = revolver;
    addRequirements(m_revolver);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_revolver.setAll(Constants.REVOLVER_DRUM_BACKWARDS_SPEED, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_revolver.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > Constants.REVOLVER_UNJAM_TIME;
  }
}
