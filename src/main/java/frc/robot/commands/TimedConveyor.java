/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorBelt;

public class TimedConveyor extends CommandBase {
  /**
   * Creates a new TimedConveyor.
   */

  double m_speed;
  double m_pulseLength;

  double m_period;

  boolean isRightEnabled;

  ConveyorBelt m_conveyorBelt;
  Timer m_mastertimer;
  Timer m_shifter; 

  public TimedConveyor(final double speed, final ConveyorBelt conveyorBelt, final double pulseLength, final double time) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_conveyorBelt = conveyorBelt;
    m_mastertimer = new Timer();
    m_period= time;

    addRequirements(m_conveyorBelt);
    m_speed = speed;
    m_pulseLength = pulseLength;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRightEnabled=!isRightEnabled;
    m_shifter.start();
    m_mastertimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_mastertimer.hasPeriodPassed(m_period)){
        if(!isRightEnabled){
          m_conveyorBelt.setLeftConveyorBelt(m_speed);
          m_conveyorBelt.stopRight();
        }
        if(isRightEnabled){
          m_conveyorBelt.setRightConveyorBelt(m_speed);
          m_conveyorBelt.stopLeft();
        }

        if (m_shifter.hasPeriodPassed(m_pulseLength)){
          isRightEnabled = !isRightEnabled;

          m_shifter.stop();
          m_shifter.reset();
          m_shifter.start();
        }
    }

    if(m_mastertimer.hasPeriodPassed(m_period)){
      m_shifter.stop();
      m_mastertimer.stop();
      m_conveyorBelt.stopAll();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
