/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.revolver.Shoot;
import frc.robot.commands.shooter.SetShooterWithLimelight;
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
            new SetShooterWithLimelight(null, shooter, turret, hood, limelight),
            sequence(
                new RunCommand(() -> driveTrain.arcadeDrive(0.3, 0.0), driveTrain).withTimeout(3),
                new Shoot(revolver, shooter, feedWheel).withTimeout(8)
            )
        );
        // @formatter:on

    }
}
