/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterHoodAngleCommand extends CommandBase {
  /**
   * Creates a new ShooterHoodAngleCommand.
   */
  private Shooter m_shooter;
  private double m_speed;
  private double m_volts;

  public ShooterHoodAngleCommand(Shooter shooter, double speed, double volts) {
    m_shooter = shooter;
    m_speed = speed;
    m_volts = volts;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setBothHoodMotorsVolts(m_volts, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopBothHoodMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
