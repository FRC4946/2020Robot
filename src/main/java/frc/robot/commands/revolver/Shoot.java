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
  private final FeedWheel m_feedWheel;

  /**
   * Starts the revolver and feedwheel if the shooter speed, turret angle, and
   * hood angle are all at the setpoints
   *
   * @param revolver  the revolver to use for this command
   * @param shooter   the shooter to use for this command
   * @param feedWheel the feedwheel to use for this command
   */
  public Shoot(Revolver revolver, Shooter shooter, FeedWheel feedWheel) {
    m_revolver = revolver;
    m_feedWheel = feedWheel;
    m_shooter = shooter;
    addRequirements(m_revolver);
  }

  @Override
  public void execute() {
    m_feedWheel.set(0.6);
    // m_revolver.set(Constants.Revolver.FORWARDS_SPEED);
    if (m_shooter.getKey()) {
      // m_feedWheel.set(0.6);
      m_revolver.set(Constants.Revolver.FORWARDS_SPEED);
    } else {
      // m_feedWheel.stop();
      m_revolver.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_revolver.stop();
    m_feedWheel.stop();
  }
}
