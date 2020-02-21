/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Revolver;
import frc.robot.subsystems.Shooter;

/**
 * Instant command to shoot
 */
public class TimedShoot extends CommandBase {

  private double m_speed, m_angle, m_time;
  private Revolver m_revolver;
  private Shooter m_shooter;
  private Hood m_hood;
  private Timer m_timer;

  /**
   * Shoots at the specified speed
   *
   * @param speed    the speed to spin the flywheel (RPM)
   * @param angle    the angle to shoot at in degrees
   * @param time     the time to shoot for in seconds
   * @param shooter  the shooter subsystem
   * @param revolver the revolver subsystem
   * @param hood     the hood subsystem
   */
  public TimedShoot(double speed, double angle, double time, Shooter shooter, Hood hood, Revolver revolver) {
    m_shooter = shooter;
    m_revolver = revolver;
    m_hood = hood;
    m_speed = speed;
    m_angle = angle;
    m_time = time;
    m_timer = new Timer();

    addRequirements(m_shooter, m_hood, m_revolver);
  }

  @Override
  public void initialize() {
    m_hood.enable();
    m_shooter.enable();
    m_hood.setSetpoint(m_angle);
    m_shooter.setSetpoint(m_speed);
    m_timer.reset();
    m_timer.start();
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
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > m_time;
  }
}
