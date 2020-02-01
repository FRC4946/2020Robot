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

public class RunConveyor extends CommandBase {

  double m_resevoirSpeed, m_feederSpeed;
  ConveyorBelt m_conveyorBelt;
  Timer m_timer;

  public RunConveyor(double resevoirSpeed, double feederSpeed, ConveyorBelt conveyorBelt) {
    m_conveyorBelt = conveyorBelt;
    m_resevoirSpeed = resevoirSpeed;
    m_feederSpeed = feederSpeed;

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
    m_conveyorBelt.runAll(m_speed);
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
