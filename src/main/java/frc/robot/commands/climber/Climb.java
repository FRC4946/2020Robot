/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class Climb extends CommandBase {

  private final Climber m_climber;
  private final Intake m_intake;
  private final DoubleSupplier m_speed;
  private final Timer m_timer;

  /**
   * Creates a new Climb command.
   */
  public Climb(DoubleSupplier speed, Climber climber, Intake intake) {
    m_climber = climber;
    m_intake = intake;
    m_speed = speed;
    m_timer = new Timer();
    addRequirements(m_climber, m_intake);
  }

  @Override
  public void initialize() {
    m_timer.start();
    m_intake.setExtended(true);
  }

  @Override
  public void execute() {
    if (m_timer.get() > 0.5) {
      m_climber.setPiston(true);
      m_climber.set(m_speed.getAsDouble());
    } else {
      m_climber.stop();
    }
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
    m_intake.setExtended(interrupted ? Value.kReverse : Value.kOff);
  }
}
