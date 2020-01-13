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
    
    m_leftConveyor = new TalonSRX(RobotMap.LEFT_CONVEYOR_BELT);
    m_rightConveyor = new TalonSRX(RobotMap.RIGHT_CONVEYOR_BELT);
    m_verticalConveyor = new TalonSRX(RobotMap.VERTICAL_CONVEYOR_BELT);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void leftConveyorBelt(double speed){
    m_leftConveyor.set(ControlMode.PercentOutput, speed);
  }

  public void rightConveyorBelt(double speed){
    m_rightConveyor.set(ControlMode.PercentOutput, speed);
  }

  public void verticalConveyorBelt(double speed){
    m_verticalConveyor.set(ControlMode.PercentOutput, speed);
  }

  public void runAll(double speed){
    leftConveyorBelt(speed);
    rightConveyorBelt(speed);
    verticalConveyorBelt(speed);
  }

  public void stopLeft(){
    leftConveyorBelt(0.0);
  }

  public void stopRight(){
    rightConveyorBelt(0.0);
  }

  public void stopVertical(){
    verticalConveyorBelt(0.0);
  }

  public void stopAll(){
    leftConveyorBelt(0.0);
    rightConveyorBelt(0.0);
    verticalConveyorBelt(0.0);
  }


}
