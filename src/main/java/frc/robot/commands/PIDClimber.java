/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;

public class PIDClimber extends PIDCommand {

  double m_height;

  public PIDClimber(final double height, final Climber climber) {
    super(new PIDController(Constants.PID_CLIMBER_P, Constants.PID_CLIMBER_I, Constants.PID_CLIMBER_D),
        // This should return the measurement
        () -> climber.getDistance(),
        // This should return the setpoint (can also be a constant)
        () -> height,
        // This uses the output
        output -> {
          climber.set(output);
        });
    addRequirements(climber);
    getController().setOutputRange(Constants.MIN_CLIMBER_SPEED, Constants.MAX_CLIMBER_SPEED);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().onTarget();
  }
}