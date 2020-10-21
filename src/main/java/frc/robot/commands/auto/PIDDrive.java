/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Utilities;

public class PIDDrive extends PIDCommand {

  double m_distance;
  double m_startDistance;
  DriveTrain m_driveTrain;

  /**
   * Creates a new PIDDrive.
   */
  public PIDDrive(double distance, double startDistance, DriveTrain driveTrain) {
    super(new PIDController(Constants.DriveTrain.DRIVE_P, Constants.DriveTrain.DRIVE_I, Constants.DriveTrain.DRIVE_D),
        () -> driveTrain.getAverageDistance() - startDistance, () -> distance, output -> {
          driveTrain.arcadeDrive(Utilities.clip(output, -0.6, 0.6), 0.0);
        });
    getController().setTolerance(Constants.DriveTrain.DRIVE_TOLERANCE);
    m_driveTrain = driveTrain;
    m_startDistance = startDistance;
    m_distance = distance;
    addRequirements(driveTrain);
  }

  public PIDDrive(double distance, DriveTrain driveTrain) {
    this(distance, 0.0, driveTrain);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return getController().atSetpoint();
    return (m_distance > 0 ? m_driveTrain.getAverageDistance() > m_startDistance + m_distance
        : m_driveTrain.getAverageDistance() < m_startDistance + m_distance);
  }
}
