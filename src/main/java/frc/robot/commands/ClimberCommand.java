/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends CommandBase {
  /**
   * Creates a new ClimberCommand.
   */

  private Climber m_climber;
  private double m_speed;
  private double m_height;
  private double m_distanceTravelled = 0.0;

  public ClimberCommand(Climber climber, double speed, double height) {
    m_climber = climber;
    m_speed = speed;
    m_height = height;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_distanceTravelled = m_climber.getDistance();

    if (m_distanceTravelled < m_height) {
      m_climber.set(m_speed);
    }
    
    if (m_distanceTravelled == m_height) {
      m_climber.set(-m_speed);
    }

    /* else {
      m_climber.stop();
    } */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_climber.getDistance() == m_height);
  }
}
