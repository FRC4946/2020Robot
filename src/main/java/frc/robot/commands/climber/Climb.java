/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class Climb extends CommandBase {

  private Climber m_climber;
  private Intake m_intake;
  private Joystick m_stick;
  private int m_axis1, m_axis2;
  private Timer m_timer = new Timer();

  /**
   * Creates a new Climb.
   */
  public Climb(Joystick stick, int axis1, int axis2, Climber climber, Intake intake) {
    m_climber = climber;
    m_intake = intake;
    m_stick = stick;
    m_axis1 = axis1;
    addRequirements(m_climber, m_intake);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_intake.setExtended(true);
  }

  @Override
  public void execute() {
    if (m_timer.get() > 0.5) {
      m_climber.setPiston(true);
      m_timer.stop();
    }
    m_climber.set(Math.sqrt(Math.pow(m_stick.getRawAxis(m_axis1), 2) + Math.pow(m_stick.getRawAxis(m_axis2), 2))
        * Constants.CLIMBER_MAX_PERCENT_OUTPUT);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_climber.stop();
    m_climber.setPiston(interrupted ? Value.kReverse : Value.kOff);
    if (interrupted) {
      m_intake.setExtended(false);
    } else {
      m_intake.setExtended(true);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
