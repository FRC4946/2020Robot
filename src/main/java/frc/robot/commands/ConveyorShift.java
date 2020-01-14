/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ConveyorBelt;


public class ConveyorShift extends CommandBase {

  double m_speed;
  double m_pulseLength;
  boolean m_isLeftRunning;
  ConveyorBelt m_conveyorBelt;
  Timer m_conveyorPulse;

  public ConveyorShift(double speed, ConveyorBelt conveyorBelt, double pulseLength) {
    m_conveyorBelt = conveyorBelt;
    m_conveyorPulse = new Timer();

    addRequirements(m_conveyorBelt);
    m_speed = speed;
    m_pulseLength = pulseLength;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // For horizontal conveyors
    m_conveyorPulse.start();
    if (m_conveyorPulse.hasPeriodPassed(m_pulseLength)) {
      if (m_isLeftRunning) {
        m_isLeftRunning = false;
        m_conveyorBelt.stopLeft();
        m_conveyorBelt.setRightConveyorBelt(m_speed);

        m_conveyorPulse.stop();
        m_conveyorPulse.reset();
      } else if (!m_isLeftRunning) {
        m_isLeftRunning = true;
        m_conveyorBelt.stopRight();
        m_conveyorBelt.setLeftConveyorBelt(m_speed);

        m_conveyorPulse.stop();
        m_conveyorPulse.reset();
      }
    }
    // For vertical conveyors
    m_conveyorBelt.setVerticalConveyorBelt(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyorBelt.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
