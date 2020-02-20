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
import frc.robot.subsystems.Revolver;
import frc.robot.subsystems.Shooter;

public class TimedShoot extends CommandBase {

  private Shooter m_shooter;
  private Revolver m_revolver;
  private double m_speed, m_angle, m_time;
  private Timer m_timer;

  /**
   * Creates a new TimedShoot command.
   */
  public TimedShoot(double speed, double angle, double time, Shooter shooter, Revolver revolver) {
    m_shooter = shooter;
    m_revolver = revolver;
    m_timer = new Timer();
    m_speed = speed;
    m_angle = angle;
    m_time = time;
    addRequirements(m_shooter, m_revolver);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_shooter.setAngleSetpoint(m_angle);
    m_shooter.setSpeedSetpoint(m_speed);
  }

  @Override
  public void execute() {
    m_revolver.setDrum(Constants.REVOLVER_DRUM_FORWARDS_SPEED);
    if (m_shooter.atSpeedSetpoint() && m_shooter.atAngleSetpoint())
      m_revolver.setFeedWheel(0.3);
    else
      m_revolver.setFeedWheel(0.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_shooter.setAngleSetpoint(Constants.HOOD_MIN_ANGLE);
    m_shooter.setSpeedSetpoint(0.0);
    m_revolver.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > m_time;
  }
}
