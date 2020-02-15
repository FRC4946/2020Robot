/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class CurvatureDrive extends CommandBase {
  /**
   * Creates a new CurvatureDrive.
   */

  DriveTrain m_driveTrain;
  double m_speed;
  double m_rotation;

  boolean m_quickturn;

  double m_distance;

  public CurvatureDrive(double speed, double distance, double rotation, DriveTrain driveTrain, boolean isquickturn) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    addRequirements(m_driveTrain);
    m_speed = speed;
    m_rotation = rotation;
    m_distance = distance; 
    m_quickturn = isquickturn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.curvatureDrive(m_speed, m_rotation, m_quickturn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.getAverageDistance() > m_distance;
  }
}
