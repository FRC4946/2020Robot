/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ConveyorBelt extends SubsystemBase {
  /**
   * Creates a new ConveyorBelt.
   */

  private TalonSRX m_ballResevoir, m_feedIntake;

  public ConveyorBelt() {
    
    m_ballResevoir = new TalonSRX(RobotMap.CAN.BALL_RESEVOIR_TALONSRX);
    m_feedIntake = new TalonSRX(RobotMap.CAN.FEED_INTAKE_TALONSRX);
  }

  /**
   * Sets the ball resevoir to the desired speed
   * @param speed the speed that the ball resevoir will run at
   */
  public void setBallResevoir(double speed){
    m_ballResevoirr.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Sets the wheel that feeds the balls to the shooter to the desired speed
   * @param speed the speed that the wheel that feeds the balls to the shooter wil run at
   */
  public void setFeedIntake(double speed){
    m_feedIntake.set(ControlMode.PercentOutput, speed);
  }


  /**
   * Sets the all components to the desired speed
   * @param speed the speed that the all components will run at
   */
  public void runAll(double speed){
    setBallResevoir(speed);
    setFeedIntake(speed);
  }

  /**
   * Stops the ball resevoir motor
   */
  public void stopBallResevoir(){
    setBallResevoir(0.0);
  }

  /**
   * Stops the wheel that feeds the balls to the shooter
   */
  public void stopFeedIntake(){
    setFeedIntake(0.0);
  }

  /**
   * Stops the all components motors
   */
  public void stopAll(){
    setBallResevoir(0.0);
    setFeedIntake(0.0);
  }


}
