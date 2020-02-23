/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.colorwheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanel;

public class TurnForDistance extends CommandBase {

  private final double m_distance;
  private final double m_speed;
  private final ControlPanel m_controlPanel;

  /**
   * Creates a new TurnForDistance command.
   */
  public TurnForDistance(double distance, double speed, ControlPanel controlPanel) {
    m_distance = distance;
    m_speed = speed;
    m_controlPanel = controlPanel;

    addRequirements(m_controlPanel);
  }

  @Override
  public void initialize() {
    m_controlPanel.resetEncoder();
  }

  @Override
  public void execute() {
    if (m_controlPanel.isExtended()) {
      m_controlPanel.set(m_speed);
    } else {
      m_controlPanel.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_controlPanel.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_controlPanel.getDistance()) > Math.abs(m_distance);
  }
}
