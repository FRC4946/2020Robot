/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoShoot extends CommandBase {
  /**
   * Creates a new AutoShoot.
   */

  Shooter m_shooter;
  Turret m_turret;
  Limelight m_limelight;
  double m_speed;

  double m_offsetX;
  double m_offsetY;


  public AutoShoot(Shooter shooter, Turret turret, Limelight limelight, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(turret);
    addRequirements(limelight);
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_offsetX=0;
    m_offsetY=0;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_limelight.getOffset()[0]+m_offsetX==0.0&&m_limelight.getOffset()[1]+m_offsetY==0.0){
      m_shooter.set(m_speed);
    }

    if(m_limelight.getOffsetX()+m_offsetX!=0){

      


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
