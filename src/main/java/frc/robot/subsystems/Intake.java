/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  /**
   * TODO: Import TalonSRX
   */

  private DoubleSolenoid m_leftElbow, m_rightElbow;
  //private Victor m_leftElbow, m_rightElbow;
  private boolean m_isLeftElbowUp, m_isRightElbowUp;

  //double m_speed;

  private TalonSRX m_spinLeft, m_spinRight;

  public Intake(double speed) {

    m_leftElbow = new DoubleSolenoid(RobotMap.LEFT_ELBOW_A, RobotMap.LEFT_ELBOW_B);
    //m_leftElbow = new Victor(RobotMap.LEFT_ELBOW_A);
    //m_isLeftElbowUp = false;

    m_rightElbow = new DoubleSolenoid(RobotMap.RIGHT_ELBOW_A, RobotMap.RIGHT_ELBOW_B);
    //m_rightElbow = new Victor(RobotMap.RIGHT_ELBOW_A);
    //m_isRightElbowUp = false;

    m_spinLeft = new TalonSRX(RobotMap.CAN.SPIN_LEFT_TALONSRX);
    m_spinRight = new TalonSRX(RobotMap.CAN.SPIN_RIGHT_TALONSRX);

    //m_speed = speed;
  }

  /** Controls the left elbow with solenoids based on a boolean
   * 
   * @param isUp moves the elbow up if true and down if false
   */
  public void setLeftElbow (boolean isUp){

    if (isUp) {
      m_leftElbow.set(Value.kForward);
      //m_leftElbow.set(m_speed);
    } 
    else {
      m_leftElbow.set(Value.kReverse);
      //m_leftElbow.set(-m_speed);
		}

		m_isLeftElbowUp = isUp;
  }
  
  /** Controls the right elbow with solenoids based on a boolean
   * 
   * @param isUp moves the elbow up if true and down if false
   */
  public void setRightElbow (boolean isUp){

    if (isUp) {
      m_rightElbow.set(Value.kForward);
      //m_rightElbow.set(m_speed);
    } 
    else {
      m_rightElbow.set(Value.kReverse);
      //m_rightElbow.set(-m_speed);
		}

		m_isRightElbowUp = isUp;
	}

  /** Moves the left elbow down using the setLeftElbow command
   * 
   */
  public void setLeftDown(){
    setLeftElbow(false);
  }

  /** Moves the left elbow down using the setLeftElbow command
   * 
   */
  public void setLeftUp(){
    setLeftElbow(true);
  }

  /** Toggles the left elbow
   * 
   */
  public void toggleLeft(){
    setLeftElbow(!m_isLeftElbowUp);
  }

  /** Retacts the left solenoid
   * 
   */
  public void offLeft(){
    m_leftElbow.set(Value.kOff);
    //m_leftElbow.set(0.0);
  }



  /** Moves the right elbow down using the setRightElbow command
   * 
   */
  public void setRightDown(){
    setRightElbow(false);
  }

  /** Moves the right elbow up using the setRightElbow command
   * 
   */
  public void setRightUp(){
    setRightElbow(true);
  }

  /** Toggles the right elbow
   * 
   */
  public void toggleRight(){
    setRightElbow(!m_isRightElbowUp);
  }

  /** Retracts the right solenoid 
   * 
   */
  public void offRight(){
    m_rightElbow.set(Value.kOff);
    //m_rightElbow.set(0.0);
  }



  /** Moves both of the elbows down using the setLeftElbow and setRightElbow command
   * 
   */
  public void setBothDown(){
    setRightElbow(false);
    setLeftElbow(false);
  }

  /** Moves both of the elbows up using the setLeftElbow and setRightElbow commands 
   * 
   */
  public void setBothUp(){
    setRightElbow(true);
    setLeftElbow(true);
  }

  /** Toggles both of the elbows up using the setLeftElbow and setRightElbow commands 
   * 
   */
  public void toggleBoth(){
    setRightElbow(!m_isRightElbowUp);
    setLeftElbow(!m_isLeftElbowUp);
  }

  /** Turns off both of the elbows up using the setLeftElbow and setRightElbow commands 
   * 
   */
  public void offBoth(){
    m_rightElbow.set(Value.kOff);
    m_leftElbow.set(Value.kOff);
    //offLeft();
    //offRight();
  }

  /** Runs the intake at the desired speed
   * 
   * @param speed the speed to run the intake at from -1 to 1
   */
  public void set(double speed){
    m_spinLeft.set(ControlMode.PercentOutput, speed);
    m_spinRight.set(ControlMode.PercentOutput, speed);
  }

  /** Stops both sets of the intake wheels
   * 
   */
  public void stopAll(){
    m_spinLeft.set(ControlMode.PercentOutput, 0.0);
    m_spinRight.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
