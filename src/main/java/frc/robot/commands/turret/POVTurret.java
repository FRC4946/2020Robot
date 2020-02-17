/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class POVTurret extends CommandBase {

  Turret m_turret;
  Shooter m_shooter;

  Joystick m_joystick;

  /**
   * Creates a new POVTurret.
   */
  public POVTurret(Joystick joystick, Turret turret, Shooter shooter) {
    m_turret = turret;
    m_shooter = shooter;
    m_joystick = joystick;
    addRequirements(turret);
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
    } else if (m_joystick.getPOV() == 90 || m_joystick.getPOV() == 45 || m_joystick.getPOV() == 135) {
      if (m_turret.getAngle() > Constants.TURRET_ROTATION_MAX) {
        m_turret.set(0.3);
      } else {
        m_turret.stop();
      }
    } else if (m_joystick.getPOV() == 180 || m_joystick.getPOV() == 135 || m_joystick.getPOV() == 225) {
      m_shooter.setAngleSetpoint(m_shooter.getAngleSetpoint() - 0.3);
    } else if (m_joystick.getPOV() == 270 || m_joystick.getPOV() == 225 || m_joystick.getPOV() == 315) {
      if (m_turret.getAngle() < Constants.TURRET_ROTATION_MIN) {
        m_turret.set(-0.3);
      } else {
        m_turret.stop();
      }
    } else if (m_joystick.getPOV() == -1) {
      m_turret.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
