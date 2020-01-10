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

/**
 * Add your docs here.
 */
public class ColorPicker extends SubsystemBase {

  ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  TalonSRX m_wheel = new TalonSRX(RobotMap.CAN.CONTROL_PANEL_TALONSRX);

  ColorMatch m_colorMatcher = new ColorMatch();

  public ColorPicker() {
    m_colorMatcher.addColorMatch(Constants.BLUE_COLOR);
    m_colorMatcher.addColorMatch(Constants.GREEN_COLOR);
    m_colorMatcher.addColorMatch(Constants.RED_COLOR);
    m_colorMatcher.addColorMatch(Constants.YELLOW_COLOR);
  }

  public void set(double speed) {
    m_wheel.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    set(0.0);
  }

  public Color getColor(){
    return m_colorSensor.getColor();
  }

  public Color getMatchingColor() {
    return m_colorMatcher.matchClosestColor(getMatchingColor()).color;
  }

}
