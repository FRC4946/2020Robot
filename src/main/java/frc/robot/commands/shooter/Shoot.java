/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Revolver;
import frc.robot.subsystems.Shooter;

/**
 * Instant command to shoot
 */
public class Shoot extends CommandBase {

  private double m_speed, m_angle;
  private Revolver m_revolver;
  private Shooter m_shooter;
  private Hood m_hood;

  /**
   * Shoots at the specified speed
   *
   * @param speed    the speed to spin the flywheel (RPM)
   * @param angle    the angle to shoot at in degrees
   * @param shooter  the shooter subsystem
   * @param revolver the revolver subsystem
   * @param hood     the hood subsystem
   */
  public Shoot(double speed, double angle, Shooter shooter, Hood hood, Revolver revolver) {
    m_shooter = shooter;
    m_revolver = revolver;
    m_hood = hood;
    m_speed = speed;
    m_angle = angle;

    addRequirements(m_shooter, m_hood, m_revolver);
  }

  @Override
  public void initialize() {
    m_hood.enable();
    m_shooter.enable();
    m_hood.setSetpoint(m_angle);
    m_shooter.setSetpoint(m_speed);
  }

  @Override
  public void execute() {
    m_revolver.setDrum(Constants.REVOLVER_DRUM_FORWARDS_SPEED);
    if (m_shooter.atSetpoint()) {
      m_revolver.setFeedWheel(0.3);
    } else {
      m_revolver.setFeedWheel(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setSetpoint(Constants.HOOD_MIN_ANGLE);
    m_shooter.setSetpoint(0.0);
    m_revolver.stop();
  }
}
