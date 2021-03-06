/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Revolver;

public class TimedRunIntake extends CommandBase {

  private final Intake m_intake;
  private final Revolver m_revolver;
  private final DoubleSupplier m_speed;
  private final double m_time;
  private final Timer m_timer;

  /**
   * Creates a new TimedRunIntake command.
   */
  public TimedRunIntake(DoubleSupplier speed, double time, Intake intake, Revolver revolver) {
    m_intake = intake;
    m_revolver = revolver;
    m_timer = new Timer();
    m_time = time;
    m_speed = speed;
    addRequirements(m_intake, m_revolver);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_intake.setExtended(true);
  }

  @Override
  public void execute() {
    m_intake.set(m_speed.getAsDouble());
    m_revolver.set(Constants.Revolver.FORWARDS_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_revolver.stop();
    m_intake.setExtended(false);
    m_intake.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > m_time;
  }
}
