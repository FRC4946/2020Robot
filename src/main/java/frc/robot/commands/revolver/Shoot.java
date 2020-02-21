/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.revolver;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Revolver;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Shoot extends CommandBase {

  private final Revolver m_revolver;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final Turret m_turret;
  private final FeedWheel m_feedWheel;

  /**
   * Starts the revolver and feedwheel if the shooter speed, turret angle, and
   * hood angle are all at the setpoints
   * 
   * @param revolver  the revolver to use for this command
   * @param shooter   the shooter to use for this command
   * @param turret    the turret to use for this command
   * @param hood      the hood to use for this command
   * @param feedWheel the feedwheel to use for this command
   */
  public Shoot(Revolver revolver, Shooter shooter, Turret turret, Hood hood, FeedWheel feedWheel) {
    m_revolver = revolver;
    m_feedWheel = feedWheel;
    m_shooter = shooter;
    m_turret = turret;
    m_hood = hood;
    addRequirements(m_revolver);
  }

  @Override
  public void execute() {
    if (m_shooter.atSetpoint() && m_hood.atSetpoint() && m_shooter.atSetpoint() && m_turret.atSetpoint()) {
      m_revolver.set(Constants.REVOLVER_DRUM_FORWARDS_SPEED);
      m_feedWheel.set(0.3);
    } else {
      m_revolver.stop();
      m_feedWheel.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_revolver.stop();
    m_feedWheel.stop();
  }
}
