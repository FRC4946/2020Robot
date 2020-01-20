/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PIDTurret extends PIDCommand {
  /**
   * Creates a new PIDTurret.
   */

  double m_xOffset;

  /**
   * 
   * @param turret  the turret subsystem
   * @param limelight   the limelight subsystem
   */
  public PIDTurret(final double degrees, final Turret turret, final Limelight limelight) {

    double[] offset = limelight.getOffset();
    m_xOffset = offset[0];

    super(
        // The controller that the command will use
        new PIDController(Constants.PID_CLIMBER_P, Constants.PID_CLIMBER_I, Constants.PID_CLIMBER_D),
        // This should return the measurement
        () -> limelight.getOffsetX(),
        // This should return the setpoint (can also be a constant)
        () -> degrees,
        // This uses the output
        output -> {
          turret.move(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    addRequirements(limelight);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (limelight.getOffsetX()==0.0 || getController().atSetpoint());
  }
}
