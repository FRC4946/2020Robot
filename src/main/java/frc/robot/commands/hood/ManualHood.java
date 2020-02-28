/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.util.Utilities;

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
    double speed = Utilities.clip(m_speedSupplier.getAsDouble(), -0.9, 0.9);
    // -0.9 to 0.9 to avoid PWM scaling errors
    if (((m_hood.getAngle() < Constants.Hood.MIN_ANGLE || Math.abs(Constants.Hood.MIN_ANGLE - m_hood.getAngle()) < 0.3)
        && speed > 0) || (m_hood.getAngle() > Constants.Hood.MAX_ANGLE && speed < 0)) {
      m_hood.stop();
    } else {
      m_hood.set(speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_hood.stop();
    m_hood.enable();
  }
}
