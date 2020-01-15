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

  boolean isRightReady;
  boolean isLeftReady;

  ConveyorBelt m_conveyorBelt;
  Timer m_timer;

  public ConveyorShift(final double speed, final ConveyorBelt conveyorBelt, final double pulseLength) {
    m_conveyorBelt = conveyorBelt;
    m_timer = new Timer();

    addRequirements(m_conveyorBelt);
    m_speed = speed;
    m_pulseLength = pulseLength;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRightReady=false;
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // For horizontal conveyors

    if(!isRightReady){
      m_conveyorBelt.setLeftConveyorBelt(m_speed);
      m_conveyorBelt.stopRight();
    }
    if(isRightReady){
      m_conveyorBelt.setRightConveyorBelt(m_speed);
      m_conveyorBelt.stopLeft();
    }

    if (m_timer.hasPeriodPassed(m_pulseLength)){
      isRightReady = !isRightReady;

      m_timer.stop();
      m_timer.reset();
      m_timer.start();
    }
    

    // For vertical conveyors
    m_conveyorBelt.setVerticalConveyorBelt(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_conveyorBelt.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
