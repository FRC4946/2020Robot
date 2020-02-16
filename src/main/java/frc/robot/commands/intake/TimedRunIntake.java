/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.intake.RunIntake.IntakeSelector;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Revolver;

public class TimedRunIntake extends CommandBase {
  
  Intake m_intake;
  Revolver m_revolver;
  double m_speed, m_time;
  IntakeSelector m_selector;
  Timer m_timer;

  /**
   * Creates a new TimedRunIntake.
   */
  public TimedRunIntake(double speed, double time, IntakeSelector selector, Intake intake, Revolver revolver) {
    m_intake = intake;
    m_revolver = revolver;
    m_time = time;
    m_speed = speed;
    m_selector = selector;
    addRequirements(m_intake, m_revolver);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    switch (m_selector) {
      case FRONT:
        m_intake.setFrontExtended(true);
        m_intake.setBackExtended(false);
        break;
      case BACK:
        m_intake.setFrontExtended(false);
        m_intake.setBackExtended(true);
        break;
      case BOTH:
      default:
        m_intake.setExtended(true);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_selector) {
      case FRONT:
        m_intake.setFront(m_speed);
        m_intake.setBack(0.0);
        break;
      case BACK:
        m_intake.setFront(0.0);
        m_intake.setBack(m_speed);
        break;
      case BOTH:
      default:
        m_intake.set(m_speed);
        break;
    }
    m_revolver.setAll(Constants.REVOLVER_DRUM_FORWARDS_SPEED, 0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_revolver.stop();
    m_intake.setExtended(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_time;
  }
}
