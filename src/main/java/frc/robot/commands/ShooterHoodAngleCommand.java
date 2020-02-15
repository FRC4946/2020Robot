/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities;
import frc.robot.subsystems.Shooter;

public class ShooterHoodAngleCommand extends CommandBase {
  /**
   * Creates a new ShooterHoodAngleCommand.
   */
  private Shooter m_shooter;
  private double m_position;
  private double m_angle;

  public ShooterHoodAngleCommand(Shooter shooter, double angle) {
    m_shooter = shooter;
    m_angle = angle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_position = Utilities.shooterPosition(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setBothHoodMotors(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopBothHoodMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_position==m_shooter.getLeftHoodPosition()) && (m_position==m_shooter.getRightHoodPosition());
  }
}
