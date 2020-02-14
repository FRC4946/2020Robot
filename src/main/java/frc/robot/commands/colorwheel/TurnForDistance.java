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
  double m_inchesTurned;
  double m_speed;

  ControlPanelWheel m_controlPanelWheel;

  /**
   * Creates a new TurnForDistance.
   */
  public TurnForDistance(double inchesTurned, double speed, ControlPanelWheel controlPanelWheel) {
    m_inchesTurned = inchesTurned;
    m_speed = speed;
    addRequirements(m_controlPanelWheel);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_controlPanelWheel.set(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanelWheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controlPanelWheel.getDistance() > m_inchesTurned;
  }
}
