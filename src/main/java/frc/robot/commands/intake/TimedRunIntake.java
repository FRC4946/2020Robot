/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class TimedRunIntake extends CommandBase {
  /**
   * Creates a new TimedRunIntake.
   */

  Intake m_intake;
  double m_intakeSpeed;
  double m_period;

  Timer m_timer;

  public TimedRunIntake(double speed, double time, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_period = time;
    m_intakeSpeed = speed;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.set(m_intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timer.hasPeriodPassed(m_period));
  }
}
