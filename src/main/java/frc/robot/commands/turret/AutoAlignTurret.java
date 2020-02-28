/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class AutoAlignTurret extends MoveTurret {

  /**
   * Creates a new AutoAlignTurret command.
   */
  public AutoAlignTurret(Turret turret, Limelight limelight) {
    super(() -> {
      return turret.getAngle() - limelight.getAngleOffset();
    }, turret);
  }
}
