/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Revolver;

public class RunIntake extends CommandBase {
  Intake m_intake;
  Revolver m_revolver;
  double m_speed;
  IntakeSelector m_selector;


  public RunIntake(double speed, IntakeSelector selector, Intake intake, Revolver revolver) {
    m_intake = intake;
    m_revolver = revolver;
    m_speed = speed;
    m_selector = selector;
    addRequirements(m_intake, m_revolver);
  }

  @Override
  public void initialize() {
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
    m_revolver.setAll(Constants.REVOLVER_DRUM_FORWARDS_SPEED, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_revolver.stop();
    m_intake.setExtended(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static enum IntakeSelector {
    FRONT,
    BACK,
    BOTH
  }
}