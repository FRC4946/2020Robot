/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class MoveTurret extends CommandBase {
  /**
   * Creates a new MoveTurret.
   */

  Turret m_turret;
  double m_speed;
  boolean m_isClockwise;
  double m_degrees;

  /**
   * 
   * @param speed the speed to set the turret motor to
   * @param turret the turret object
   * @param clockwise indicates the direction the turret moves
   * @param degrees the degrees the turret should move
   */

  public MoveTurret(double speed, double degrees, Turret turret, boolean clockwise) {
    m_turret = turret;
    m_speed = speed;
    m_isClockwise = clockwise;
    m_degrees = degrees;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_isClockwise){
      m_turret.move(m_speed);
    }
    else if(!m_isClockwise){
      m_turret.move(-m_speed);
    } 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_turret.getAngle()) > Math.abs(m_degrees);
  }
}