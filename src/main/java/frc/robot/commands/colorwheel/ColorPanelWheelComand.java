/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.colorwheel;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ControlPanelWheel;

public class ColorPanelWheelComand extends CommandBase {
  /**
   * Creates a new ColorPanelWheelComand.
   */
  
  ControlPanelWheel m_controlPanelWheel;
  Color m_currentColor;
  Color m_targetColor;
  double m_speed;

  public ColorPanelWheelComand(ControlPanelWheel controlPanelWheel, Color targetColor, Color currentColor, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controlPanelWheel = controlPanelWheel;
    m_targetColor = targetColor;
    m_currentColor = currentColor;
    m_speed = speed;

    addRequirements(m_controlPanelWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentColor = m_controlPanelWheel.getClosestMatch().color;

    if (m_currentColor != m_targetColor) {
      m_controlPanelWheel.set(m_speed);  
      m_currentColor = m_controlPanelWheel.getClosestMatch().color;
    } 
    
    else {
      m_controlPanelWheel.stopWheel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanelWheel.stopWheel();
    m_controlPanelWheel.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}