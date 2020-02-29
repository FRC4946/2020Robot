/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.commands.shooter.SetShooterWithLimelightToSpeed;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Revolver;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class DriveAndShoot extends ParallelCommandGroup {

  public DriveAndShoot(DriveTrain driveTrain, FeedWheel feedWheel, Hood hood, Limelight limelight, Revolver revolver,
      Shooter shooter, Turret turret) {

    // @formatter:off
    addCommands(
        new SetShooterWithLimelightToSpeed(3000, shooter, turret, hood, limelight),
        sequence( 
            new RunCommand(() -> driveTrain.arcadeDrive(0.5, 0.0), driveTrain).withTimeout(2).andThen(new RunCommand(() -> {driveTrain.arcadeDrive(-0.1, 0.0);}, driveTrain).withTimeout(0.1)),
            new RunCommand(() -> {}).withTimeout(6),
            new RunCommand(() -> {
              revolver.set(Constants.Revolver.FORWARDS_SPEED);
              feedWheel.set(0.6);
            }, revolver, feedWheel)
        )
    );
    // @formatter:on
    
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
