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

public class MoveTurret extends CommandBase {

  private final Turret m_turret;
  private final DoubleSupplier m_target;

  /**
   * Creates a new MoveTurret command.
   */
  public MoveTurret(DoubleSupplier target, Turret turret) {
    m_target = target;
    m_turret = turret;
    addRequirements(m_turret);
  }

  @Override
  public void execute() {
    m_turret.setSetpoint(m_target.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return m_turret.atSetpoint();
  }
}
