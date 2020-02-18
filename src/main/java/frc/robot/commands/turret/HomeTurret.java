/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class HomeTurret extends PIDCommand {

  private Turret m_turret;
  private Shooter m_shooter;

  /**
   * Creates a new HomeTurret.
   */
  public HomeTurret(Shooter shooter, Turret turret) {
    super(new PIDController(Constants.PID_TURRET_P, Constants.PID_TURRET_I, Constants.PID_TURRET_D),

        () -> turret.getAngle(),

        () -> Constants.TURRET_HOME_ANGLE,

        output -> {
          output += (output > 0 ? Constants.PID_TURRET_OFFSET : -Constants.PID_TURRET_OFFSET);
          output = (output < 0 ? -1 : 1) * Math.min(Math.abs(output), 1.0);
          turret.set(Constants.TURRET_MAX_PERCENT * (output));
        });
    m_turret = turret;
    m_shooter = shooter;
    addRequirements(m_shooter, m_turret);
    getController().setTolerance(Constants.TURRET_PID_TOLERANCE);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_shooter.setSpeedSetpoint(0.0);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_shooter.setAngleSetpoint(Constants.HOOD_MIN_ANGLE);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getController().getVelocityError()) < Constants.TURRET_VELOCITY_ERROR_THRESHOLD
        && getController().atSetpoint();
  }
}
