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
/**
 * Turns the control panel until the color sensor is over a specified color
 * @author jacob
 */
public class TurnToColor extends CommandBase {

  double m_speed;
  Color m_color;
  ControlPanel m_controlPanel;

  /**
   * Turns the control panel wheel, until the color sensor is over a specific color.
   */
  public TurnToColor(double speed, Color color, ControlPanel controlPanelWheel) {
    m_speed = speed;
    m_color = color;
    m_controlPanel = controlPanelWheel;
    addRequirements(m_controlPanel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_controlPanel.set(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controlPanel.getCurrentColor().color.equals(m_color);
  }
}
