/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.revolver;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Revolver;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Shoot extends CommandBase {

  Revolver m_revolver;
  Shooter m_shooter;
  Hood m_hood;
  Turret m_turret;

  /**
   * Starts the revolver and feedwheel if the shooter speed, turret angle, and
   * hood angle are all at the setpoints
   * 
   * 
   * @param revolver the revolver to use for this command
   * @param shooter  the revolver to use for this subsystem
   * @param turret   the revolver to use for this subsystem
   * @param hood     the revolver to use for this subsystem
   */
  public Shoot(Revolver revolver, Shooter shooter, Turret turret, Hood hood) {
    m_revolver = revolver;
    addRequirements(m_revolver);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (m_shooter.atSetpoint() && m_hood.atSetpoint() && m_shooter.atSetpoint()) {
      m_revolver.setAll(Constants.REVOLVER_DRUM_FORWARDS_SPEED, 0.3);
    } else {
      m_revolver.setAll(0.0, 0.0);
    }
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
