/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Revolver;

public class RunIntake extends CommandBase {

  private final Intake m_intake;
  private final Revolver m_revolver;
  private final DoubleSupplier m_speed;

  /**
   * Creates a new RunIntake command.
   */
  public RunIntake(DoubleSupplier speed, Intake intake, Revolver revolver) {
    m_intake = intake;
    m_revolver = revolver;
    m_speed = speed;
    addRequirements(m_intake, m_revolver);
  }

  @Override
  public void initialize() {
    m_intake.setExtended(true);
  }

  @Override
  public void execute() {
    m_intake.set(m_speed.getAsDouble());
    m_revolver.set(Constants.Revolver.FORWARDS_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    m_revolver.stop();
    m_intake.setExtended(false);
    m_intake.stop();
  }
}
