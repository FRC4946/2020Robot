/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SDrive extends CommandBase {

  DriveTrain m_driveTrain;
  double m_distance, m_startDistance, m_speed;

  /**
   * Creates a new SDrive.
   */
  public SDrive(double distance, double speed, DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    m_distance = distance;
    m_speed = speed;
    addRequirements(m_driveTrain);
  }

  @Override
  public void initialize() {
    m_startDistance = m_driveTrain.getAverageDistance();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.arcadeDrive(m_speed, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_distance > 0 ? m_driveTrain.getAverageDistance() > m_startDistance + m_distance
        : m_driveTrain.getAverageDistance() < m_startDistance + m_distance;
  }
}
