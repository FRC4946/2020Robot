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
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAtSpeed extends PIDCommand {

  /**
   * Creates a new ShootAtSpeed
   */
  public ShootAtSpeed(Shooter shooter) {
    super(
        // The controller that the command will use
        new PIDController(Constants.SHOOTER_VELOCITY_CONTROL_P, Constants.SHOOTER_VELOCITY_CONTROL_I,
            Constants.SHOOTER_VELOCITY_CONTROL_D),
        // This should return the measurement
        () -> shooter.getAverageSpeed(),
        // This should return the setpoint (can also be a constant)
        () -> shooter.getSetpoint(),
        // This uses the output
        output -> {
          shooter.set(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(shooter);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
