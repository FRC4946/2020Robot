/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ManualTurret extends CommandBase {

  private final Turret m_turret;
  private final DoubleSupplier m_speedSupplier;

  /**
   * Creates a new POVTurret.
   */
  public ManualTurret(DoubleSupplier speedSupplier, Turret turret) {
    m_turret = turret;
    m_speedSupplier = speedSupplier;
    addRequirements(m_turret);
  }

  @Override
  public void execute() {
    m_turret.set(m_speedSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }
}
