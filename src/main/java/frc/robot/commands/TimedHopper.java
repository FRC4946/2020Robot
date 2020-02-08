/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedHopper extends CommandBase {
  /**
   * Creates a new TimedHopper.
   */
  double m_drumSpeed, m_feedWheelSpeed;

  double m_period;

  Hopper m_hopper;
  Timer m_timer;

  public TimedHopper(final double drumSpeed,final double feedWheelSpeed, final Hopper hopper, final double timeSpentBackwards) {
     
    m_timer = new Timer();
    m_period = timeSpentBackwards;

    m_hopper = hopper;
    addRequirements(m_hopper);
    m_drumSpeed = drumSpeed;
    m_feedWheelSpeed = feedWheelSpeed;

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
    m_hopper.setAll(m_drumSpeed, m_feedWheelSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_period;
  }
}
