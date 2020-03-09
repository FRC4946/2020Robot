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
                new PIDDrive(2.6416, driveTrain).withTimeout(3)
              ),
              new PIDDrive(-1.016, driveTrain).withTimeout(2),
              new PIDTurn(-100, driveTrain).withTimeout(1.5)
          )
        ),
        parallel(
          new SetShooterWithLimelightToSpeed(3000, shooter, turret, hood, limelight),
          sequence(
            new PIDDrive(3.556, driveTrain).withTimeout(5),
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
