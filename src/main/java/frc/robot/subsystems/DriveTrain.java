/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {
  private SpeedControllerGroup m_leftSide, m_rightSide;

  public DriveTrain(){
    m_leftSide = new SpeedControllerGroup(null);
    m_rightSide = new SpeedControllerGroup(null);
  }
}
