/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class TimedClimber extends CommandBase {
  /**
   * Creates a new TimedClimber.
   */

  private Climber m_climber;
  private Timer m_climberTimer;
  private double m_time;
  private double m_speed;
  private boolean m_hasTimePassed;

  public TimedClimber(Climber climber, double speed, double time) {
    m_climber = climber;
    m_time = time;
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climberTimer = new Timer();
    m_climberTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hasTimePassed = m_climberTimer.hasPeriodPassed(m_time);

    if (!m_hasTimePassed) {
      m_climber.set(m_speed);
    } else {
      m_climber.stop();
      m_climberTimer.stop();
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
