/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.shooter.SetShooterWithLimelightToSpeed;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Revolver;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class MiddlePickupAndShoot extends SequentialCommandGroup {

  public MiddlePickupAndShoot(DriveTrain driveTrain, FeedWheel feedWheel, Hood hood, Intake intake, Limelight limelight,
      Revolver revolver, Shooter shooter, Turret turret) {

    // @formatter:off
    addCommands(
        parallel(
          new InstantCommand(() -> {
            shooter.setSetpoint(3000);
            shooter.enable();
          }, shooter),
          sequence(
            new InstantCommand(() -> intake.setExtended(true), intake), // intake out
              parallel(
                new RunCommand(() -> {
                  revolver.set(2 * Constants.Revolver.FORWARDS_SPEED);
                  intake.set(1.0);
                }, revolver, intake), // intake
                new PIDDrive(1.75, driveTrain)
              ),
              new PIDDrive(-0.5, driveTrain),
              new PIDTurn(-120, driveTrain),
              new PIDDrive(2, driveTrain)
          )
        ),
        parallel(
          new SetShooterWithLimelightToSpeed(3000, shooter, turret, hood, limelight),
          sequence(
            new RunCommand(() -> {}).withTimeout(3),
            new RunCommand(() -> {
              revolver.set(Constants.Revolver.FORWARDS_SPEED);
              feedWheel.set(0.6);
            }, revolver, feedWheel)
          )
        )
    );
    // @formatter:on

  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
