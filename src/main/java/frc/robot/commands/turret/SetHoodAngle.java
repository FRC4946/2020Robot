/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetHoodAngle extends CommandBase {
  /**
   * Creates a new SetHoodAngle.
   */

  Shooter m_shooter;
  Joystick m_joystick;

  public SetHoodAngle(Joystick joystick, Shooter shooter) {
    m_shooter = shooter;
    m_joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setEnabledHood(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_joystick.getPOV() == 0 || m_joystick.getPOV() == 45 || m_joystick.getPOV() == 315) {
      m_shooter.setAngleSetpoint(m_shooter.getAngleSetpoint() + 0.3);
    } else if (m_joystick.getPOV() == 180 || m_joystick.getPOV() == 135 || m_joystick.getPOV() == 225) {
      m_shooter.setAngleSetpoint(m_shooter.getAngleSetpoint() - 0.3);
    }
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
