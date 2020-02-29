/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * Add your docs here.
 */
public class SetShooterWithLimelightToSpeed extends SetShooterWithLimelight {

    private final double m_speed;
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Hood m_hood;
    private final Limelight m_limelight;

    public SetShooterWithLimelightToSpeed(double speed, Shooter shooter, Turret turret, Hood hood,
            Limelight limelight) {
        super(shooter, turret, hood, limelight);
        m_shooter = shooter;
        m_limelight = limelight;
        m_turret = turret;
        m_hood = hood;
        m_speed = speed;
    }

    @Override
    public void execute() {
        if (!m_hood.isEnabled())
            m_hood.enable();

        if (!m_shooter.isEnabled())
            m_shooter.enable();

        m_turret.setSetpoint(m_turret.getAngle() - m_limelight.getAngleOffset());

        m_shooter.setSetpoint(m_speed);
        m_hood.setSetpoint(m_limelight.getHoodAngle());

        if (m_shooter.atSetpoint() && m_hood.atSetpoint() && m_turret.atSetpoint()) {
            m_shooter.setKey(true);
        } else {
            m_shooter.setKey(false);
        }
    }
}
