/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.colorwheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelWheel;

public class TurnForDistance extends CommandBase {
  double m_distance;
  double m_speed;
  boolean m_clockwise;
  ControlPanelWheel m_controlPanelWheel;

  /**
   * Creates a new TurnForDistance.
   */
  public TurnForDistance(double distance, double speed, ControlPanelWheel controlPanelWheel, boolean clockwise) {
    m_distance = distance;
    m_speed = speed;
    m_clockwise = clockwise;
    addRequirements(m_controlPanelWheel);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controlPanelWheel.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_clockwise){
      if(m_controlPanelWheel.getDistance()<m_distance){
        m_controlPanelWheel.set(m_speed);
      }
    }
    else if(!m_clockwise){
      if(m_controlPanelWheel.getDistance()>m_distance){
        m_controlPanelWheel.set(-m_speed);
      }
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanelWheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_clockwise) ? m_controlPanelWheel.getDistance()>m_distance : m_controlPanelWheel.getDistance()<m_distance;
  }
}
