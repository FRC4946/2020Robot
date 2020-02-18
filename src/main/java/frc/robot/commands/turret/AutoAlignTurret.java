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
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class AutoAlignTurret extends PIDCommand {

  private Turret m_turret;
  private Limelight m_limelight;

  /**
   * Creates a new AutoAlignTurret.
   */
  public AutoAlignTurret(Turret turret, Limelight limelight) {
    super(new PIDController(Constants.PID_TURRET_P, Constants.PID_TURRET_I, Constants.PID_TURRET_D),
        () -> turret.getAngle(), 
        
        () -> {
          double angle = (turret.getAngle() + limelight.getAngleOffset());
          angle = Math.max(Constants.TURRET_ROTATION_MIN, angle);
          angle = Math.min(Constants.TURRET_ROTATION_MAX, angle);

          return angle;
        }, 
        
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
  }
}
