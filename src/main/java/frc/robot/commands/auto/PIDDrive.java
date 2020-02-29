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

public class PIDDrive extends PIDCommand {
  /**
   * Creates a new PIDDrive.
   */
  public PIDDrive(double distance, double startDistance, DriveTrain driveTrain) {
    super(
      new PIDController(Constants.DriveTrain.DRIVE_P, Constants.DriveTrain.DRIVE_I, Constants.DriveTrain.DRIVE_D),
        () -> driveTrain.getAverageDistance() - startDistance, 
        () -> distance, 
        output -> {
          driveTrain.arcadeDrive(output, 0.0);
        });
    getController().setTolerance(Constants.DriveTrain.DRIVE_TOLERANCE);
    addRequirements(driveTrain);
  }

  public PIDDrive(double distance, DriveTrain driveTrain) {
    this(distance, 0.0, driveTrain);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
