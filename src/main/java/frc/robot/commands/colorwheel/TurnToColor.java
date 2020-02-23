/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.colorwheel;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanel;

public class TurnToColor extends CommandBase {

  private final double m_speed;
  private final Color m_color;
  private final ControlPanel m_controlPanel;

  /**
   * Turns the control panel until the color sensor detects the specified color.
   */
  public TurnToColor(double speed, Color color, ControlPanel controlPanelWheel) {
    m_speed = speed;
    m_color = color;
    m_controlPanel = controlPanelWheel;

    addRequirements(m_controlPanel);
  }

  @Override
  public void initialize() {
    if (!m_controlPanel.isExtended()) {
      cancel();
    }
  }

  @Override
  public void execute() {
    m_controlPanel.set(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_controlPanel.stop();
  }

  @Override
  public boolean isFinished() {
    return m_controlPanel.getCurrentColor().color.equals(m_color) || !m_controlPanel.isExtended();
  }
}
