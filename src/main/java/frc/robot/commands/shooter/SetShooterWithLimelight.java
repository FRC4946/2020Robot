/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * Command to shoot
 */
public class SetShooterWithLimelight extends CommandBase {

  private final Shooter m_shooter;
  private final Turret m_turret;
  private final Hood m_hood;
  private final Limelight m_limelight;

  private final Joystick m_joystick;
  private final DoubleSupplier m_manualTurretSupplier;

  /**
   * Uses the limelight to calculate shooter speed, hood angle, and turret angle,
   * moves to these calculated values
   *
   * @param joystick  the joystick to vibrate when the shooter is at setpoint
   * @param shooter   the shooter to use for this command
   * @param turret    the turret to use for this command
   * @param hood      the hood to use for this command
   * @param limelight the limelight to use for this command
   */
  public SetShooterWithLimelight(Shooter shooter, Turret turret, Hood hood, Limelight limelight) {
    this(shooter, turret, hood, limelight, null, null);
  }

  public SetShooterWithLimelight(Shooter shooter, Turret turret, Hood hood, Limelight limelight, Joystick joystick, DoubleSupplier manualTurretSupplier) {
    m_shooter = shooter;
    m_turret = turret;
    m_hood = hood;
    m_limelight = limelight;
    m_joystick = joystick;
    m_manualTurretSupplier = manualTurretSupplier;

    addRequirements(m_shooter, m_turret, m_hood);
  }

  @Override
  public void initialize() {
    m_hood.enable();
    m_shooter.enable();
  }

  @Override
  public void execute() {
    if (!m_hood.isEnabled())
      m_hood.enable();

    if (!m_shooter.isEnabled())
      m_shooter.enable();

    if (!m_limelight.getHasTarget() && m_manualTurretSupplier != null) {
      m_turret.set(m_manualTurretSupplier.getAsDouble());
    } else {
      m_turret.setSetpoint(m_turret.getAngle() - m_limelight.getAngleOffset());
    }

    m_shooter.setSetpoint(m_limelight.getShooterSpeed());
    m_hood.setSetpoint(m_limelight.getHoodAngle());

    if (m_shooter.atSetpoint() && m_hood.atSetpoint() && m_turret.atSetpoint()) {
      if (m_joystick != null) {
        m_joystick.setRumble(RumbleType.kLeftRumble, 0.7);
        m_joystick.setRumble(RumbleType.kRightRumble, 0.7);
      }
      m_shooter.setKey(true);
    } else {
      if (m_joystick != null) {
        m_joystick.setRumble(RumbleType.kLeftRumble, 0.0);
        m_joystick.setRumble(RumbleType.kRightRumble, 0.0);
      }
      m_shooter.setKey(false);
    }
  }
}
