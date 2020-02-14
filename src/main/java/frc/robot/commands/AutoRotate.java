/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Utilities;
import frc.robot.subsystems.DriveTrain;

public class AutoRotate extends CommandBase {
  double m_angle;
  boolean m_turnLeft;

  DriveTrain m_driveTrain;

  public AutoRotate(double angle, DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    m_angle = Utilities.conformAngle(angle);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnLeft = ((m_angle - m_driveTrain.getGyroAngle()) > 0 == Math
        .abs(m_angle - m_driveTrain.getGyroAngle()) <= 180);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_turnLeft) {
      m_driveTrain.tankDrive(-0.3, 0.3);
    } else {
      m_driveTrain.tankDrive(0.3, -0.3);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_driveTrain.getGyroAngle() - m_angle) < 2);
  }
}
