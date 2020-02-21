/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class ManualHood extends CommandBase {

  private final Hood m_hood;
  private final DoubleSupplier m_speedSupplier;

  /**
   * Creates a new ManualHood command.
   */
  public ManualHood(DoubleSupplier speedSupplier, Hood hood) {
    m_hood = hood;
    m_speedSupplier = speedSupplier;
    addRequirements(m_hood);
  }

  @Override
  public void initialize() {
    m_hood.disable();
  }

  @Override
  public void execute() {
    m_hood.set(m_speedSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_hood.stop();
    m_hood.enable();
  }
}
