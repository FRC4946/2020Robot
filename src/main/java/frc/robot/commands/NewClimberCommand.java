/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class NewClimberCommand extends CommandBase {
  /**
   * Creates a new NewClimberCommand.
   */
  Climber m_climber;
  double m_speed;
  double m_distance;
  int m_timesAtBottom;

  public NewClimberCommand(Climber climber, double speed) {
    m_climber = climber;
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_distance = Constants.CLIMBER_POT_BOTTOM;
    m_timesAtBottom = 0;
    m_climber.setPiston(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_distance = m_climber.getDistance();
    if (m_distance == Constants.CLIMBER_POT_BOTTOM) {
      m_timesAtBottom++;
    }

    m_climber.set(m_speed);

    if (Constants.CLIMBER_DISTANCE_TO_TOP - m_distance + Constants.CLIMBER_POT_BOTTOM == 0) {
      m_climber.setPiston(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setPiston(false);
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timesAtBottom == 2;
  }
}
