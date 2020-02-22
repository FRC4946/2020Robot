/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class FollowTrajectory extends RamseteCommand {

  private final DriveTrain m_driveTrain;

  /**
   * Follows a specified trajectory until the estimated time to complete the
   * trajectory has passed
   * 
   * @param trajectory the trajectory to follow
   * @param driveTrain the drivetrain to use with the command
   */
  public FollowTrajectory(Trajectory trajectory, DriveTrain driveTrain) {
    super(trajectory, () -> driveTrain.getPose(),
        new RamseteController(Constants.DriveTrain.RAMSETE_B, Constants.DriveTrain.RAMSETE_ZETA),
        driveTrain.getKinematics(), (leftSpeed, rightSpeed) -> {
          driveTrain.setLeftVelocity(leftSpeed);
          driveTrain.setRightVelocity(rightSpeed);
        }, driveTrain);
    m_driveTrain = driveTrain;
    m_driveTrain.resetControllers();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_driveTrain.stop();
  }
}
