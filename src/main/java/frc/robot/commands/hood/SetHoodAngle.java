/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class SetHoodAngle extends CommandBase {
  private final Hood m_hood;
  private final double m_speed;

  /**
   * Creates a new SetHoodAngle command.
   */

  public SetHoodAngle(double speed, Hood hood) {
    m_speed = speed;
    m_hood = hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hood.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hood.setSetpoint(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
