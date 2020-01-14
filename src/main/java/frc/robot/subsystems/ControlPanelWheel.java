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

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ControlPanelWheel extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  public TalonSRX m_wheelMotor = new TalonSRX(RobotMap.CAN.CONTROL_PANEL_TALONSRX);
  public ColorMatch m_colorMatcher = new ColorMatch();

  public ControlPanelWheel() {
    m_colorMatcher.addColorMatch(Constants.COLOR_BLUE);
    m_colorMatcher.addColorMatch(Constants.COLOR_GREEN);
    m_colorMatcher.addColorMatch(Constants.COLOR_RED);
    m_colorMatcher.addColorMatch(Constants.COLOR_YELLOW);
  }

  public void set(double speed){
    m_wheelMotor.set(ControlMode.PercentOutput, speed);
  }

  public Color getMatchedColor(){
    return m_colorMatcher.matchClosestColor(m_colorSensor.getColor()).color;
  }

  public static ColorMatchResult matchresult = getMatchedColor();


}
