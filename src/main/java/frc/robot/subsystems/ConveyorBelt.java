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

  private TalonSRX m_leftConveyor, m_rightConveyor, m_verticalConveyor;

  public ConveyorBelt() {
    
    m_leftConveyor = new TalonSRX(RobotMap.CAN.LEFT_CONVEYOR_BELT_TALONSRX);
    m_rightConveyor = new TalonSRX(RobotMap.CAN.RIGHT_CONVEYOR_BELT_TALONSRX);
    m_verticalConveyor = new TalonSRX(RobotMap.CAN.VERTICAL_CONVEYOR_BELT_TALONSRX);
  }

  /**
   * Sets the left conveyor to the desired speed
   * @param speed the speed that the left conveyor will run at
   */
  public void setLeftConveyorBelt(double speed){
    m_leftConveyor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Sets the right conveyor to the desired speed
   * @param speed the speed that the right conveyor will run at
   */
  public void setRightConveyorBelt(double speed){
    m_rightConveyor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Sets the vertical conveyor to the desired speed
   * @param speed the speed that the vertical conveyor will run at
   */
  public void setVerticalConveyorBelt(double speed){
    m_verticalConveyor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Sets the all conveyors to the desired speed
   * @param speed the speed that the all conveyors will run at
   */
  public void runAll(double speed){
    setLeftConveyorBelt(speed);
    setRightConveyorBelt(speed);
    setVerticalConveyorBelt(speed);
  }

  /**
   * Stops the left conveyor motor
   */
  public void stopLeft(){
    setLeftConveyorBelt(0.0);
  }

  /**
   * Stops the right conveyor motor
   */
  public void stopRight(){
    setRightConveyorBelt(0.0);
  }

  /**
   * Stops the vertical conveyor motor
   */
  public void stopVertical(){
    setVerticalConveyorBelt(0.0);
  }

  /**
   * Stops the all conveyor motors
   */
  public void stopAll(){
    setLeftConveyorBelt(0.0);
    setRightConveyorBelt(0.0);
    setVerticalConveyorBelt(0.0);
  }


}
