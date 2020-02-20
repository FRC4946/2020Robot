/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {

  private Climber m_climber;
  private double m_speed;

  /**
   * Creates a new Climb command.
   */
  public Climb(double speed, Climber climber) {
    m_climber = climber;
    m_speed = speed;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climber.setPiston(true);
  }

  @Override
  public void execute() {
    m_climber.set(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
    m_climber.setPiston(Value.kOff);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
