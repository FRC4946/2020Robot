/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class AutoAlignTurret extends MoveTurret {

  /**
   * Creates a new AutoAlignTurret command.
   */
  public AutoAlignTurret(Turret turret, Limelight limelight) {
<<<<<<< HEAD
    super(() -> turret.getAngle() + limelight.getAngleOffset(), turret);
=======
    super(new PIDController(Constants.PID_TURRET_P, Constants.PID_TURRET_I, Constants.PID_TURRET_D),
        () -> turret.getAngle(),

        () -> turret.getAngle() + limelight.getAngleOffset(),

        output -> {
          output += (output > 0 ? Constants.PID_TURRET_OFFSET : -Constants.PID_TURRET_OFFSET);
          output = (output < 0 ? -1 : 1) * Math.min(Math.abs(output), 1.0);
          turret.set(Constants.TURRET_MAX_PERCENT * (output));
        });

    m_limelight = limelight;
    m_turret = turret;

    addRequirements(m_turret, m_limelight);
    getController().setTolerance(Constants.TURRET_PID_TOLERANCE);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getController().getVelocityError()) < Constants.TURRET_VELOCITY_ERROR_THRESHOLD
        && getController().atSetpoint();
>>>>>>> Implement requested changes
  }
}
