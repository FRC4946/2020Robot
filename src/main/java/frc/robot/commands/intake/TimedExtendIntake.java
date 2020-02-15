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

public class TimedExtendIntake extends CommandBase {
  /**
   * Creates a new TimedExtendIntake.
   */
  Intake m_intake;
  Timer m_timer;
  double m_period;
  boolean m_hasTimePassed;

  public TimedExtendIntake(Intake intake, double speed, double time) {
    m_intake = intake;
    m_period = time;
    m_timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hasTimePassed = false;
    m_timer.reset();
    m_timer.start();

    m_intake.setBothDown();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasPeriodPassed(m_period)) {
      m_hasTimePassed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setBothUp();
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hasTimePassed;
  }
}
