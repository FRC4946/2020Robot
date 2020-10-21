/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.commands.shooter.SetShooterWithLimelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Revolver;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class TrenchAuto extends ParallelCommandGroup {
  /**
   * Creates a new TrenchAuto.
   */
  public TrenchAuto(DriveTrain driveTrain, Revolver revolver, Intake intake, Shooter shooter, Turret turret, Hood hood, Limelight limelight, FeedWheel feedWheel) {

    addCommands(
      new InstantCommand(() -> intake.setExtended(true)),
      new RunCommand(() -> revolver.set(Constants.Revolver.FORWARDS_SPEED), revolver),
      new RunCommand(() -> intake.set(1.0), intake),
      sequence(
        new SDrive(3.048, 0.7, driveTrain).andThen(() -> driveTrain.arcadeDrive(-0.1, 0.0), driveTrain).withTimeout(0.1),
        new RunCommand(() -> {}).withTimeout(0.75),
        new InstantCommand(() -> {
          shooter.setSetpoint(4600);
          shooter.enable();
        }, shooter),
        new SDrive(-3.048, -0.7, driveTrain).andThen(() -> driveTrain.arcadeDrive(0.1, 0.0), driveTrain).withTimeout(0.1),
        new PIDTurn(160, driveTrain).withTimeout(1.5),
        parallel(
          new SetShooterWithLimelight(shooter, turret, hood, limelight),
          sequence(
            new RunCommand(() -> {}).withTimeout(3),
            new RunCommand(() -> {
              // revolver.set(Constants.Revolver.FORWARDS_SPEED);
              feedWheel.set(0.6);
            }, feedWheel)
          )
        )
      )
    );
  }
}
