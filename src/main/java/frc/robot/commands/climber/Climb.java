/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {

  private Climber m_climber;
  private DoubleSupplier m_speed;

  /**
   * Creates a new Climb command.
   */
  public Climb(DoubleSupplier speed, Climber climber) {
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
    m_climber.set(m_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stop();

    // If the robot is disabled after climbing at the end of the match, the
    // interrupted flag will be set false,
    // If the button this command is assigned to is pressed again, interrupted will
    // be true,
    // This means that if the robot is disabled after climbing, the climber will
    // stay up, but if the button is pressed again the climber will come down
    m_climber.setPiston(interrupted ? Value.kReverse : Value.kOff);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
