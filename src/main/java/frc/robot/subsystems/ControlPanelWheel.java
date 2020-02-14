/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ControlPanelWheel extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private ColorSensorV3 m_colorSensor;
  private TalonSRX m_wheelMotor;
  private ColorMatch m_colorMatcher;
  private Encoder m_wheelEncoder;

  public ControlPanelWheel() {
    m_colorSensor = new ColorSensorV3(Port.kOnboard);
    m_wheelMotor = new TalonSRX(RobotMap.CAN.CONTROL_PANEL_TALONSRX);
    m_colorMatcher = new ColorMatch();
    m_wheelEncoder = new Encoder(RobotMap.DIO.WHEEL_ENCODER_A, RobotMap.DIO.WHEEL_ENCODER_B);

    m_colorMatcher.addColorMatch(Constants.COLOR_BLUE);
    m_colorMatcher.addColorMatch(Constants.COLOR_GREEN);
    m_colorMatcher.addColorMatch(Constants.COLOR_RED);
    m_colorMatcher.addColorMatch(Constants.COLOR_YELLOW);
  }

  /**
   * Moves the color wheel
   * 
   * @param speed the speed of the wheel motor
   */

  public void set(double speed) {
    m_wheelMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stops the wheel motor
   */
  public void stop() {
    set(0.0);
  }

  public double getDistance() {
    return m_wheelEncoder.getDistance();
  }

  public void resetEncoder() {
    m_wheelEncoder.reset();
  }

  public Color getDetectedColor() {
    return m_colorSensor.getColor();
  }

  public ColorMatchResult getClosestMatch() {
    return m_colorMatcher.matchClosestColor(getDetectedColor());
  }
}
