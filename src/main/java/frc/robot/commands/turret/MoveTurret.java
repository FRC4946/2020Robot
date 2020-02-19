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
import frc.robot.util.Utilities;

public class MoveTurret extends PIDCommand {

  private Turret m_turret;
  private Shooter m_shooter;

  /**
   * Creates a new MoveTurret.
   */
  public MoveTurret(double angle, Shooter shooter, Turret turret) {
    super(new PIDController(Constants.PID_TURRET_P, Constants.PID_TURRET_I, Constants.PID_TURRET_D),

        () -> turret.getAngle(),

        () -> Utilities.clip(angle, Constants.TURRET_ROTATION_MIN, Constants.TURRET_ROTATION_MAX),

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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
