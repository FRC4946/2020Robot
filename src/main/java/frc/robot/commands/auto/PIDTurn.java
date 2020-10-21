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

public class PIDTurn extends PIDCommand {

  DriveTrain m_driveTrain;

  /**
   * Creates a new PIDTurn.
   */
  public PIDTurn(double setpoint, DriveTrain driveTrain) {
    super(
        new PIDController(Constants.DriveTrain.TURN_P, Constants.DriveTrain.TURN_I, Constants.DriveTrain.TURN_D),
        () -> driveTrain.getGyroAngle(),
        () -> setpoint,
        output -> {
          driveTrain.arcadeDrive(0.0, Utilities.clip(-output, -0.6, 0.6));
        });
    getController().setTolerance(Constants.DriveTrain.TURN_TOLERANCE);
    addRequirements(driveTrain);
    m_driveTrain = driveTrain;
  }

  @Override
  public void initialize() {
    super.initialize();
    //TODO : Make this no longer require a complete reset of the drivetrain
    m_driveTrain.resetDriveTrain();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
