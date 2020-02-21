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

  private ColorSensorV3 m_sensor;
  private TalonSRX m_wheel;
  private ColorMatch m_matcher;
  private Encoder m_encoder;

  public ControlPanelWheel() {
    m_sensor = new ColorSensorV3(Port.kOnboard);
    m_wheel = new TalonSRX(RobotMap.CAN.CONTROL_PANEL_TALONSRX);
    m_matcher = new ColorMatch();
    m_encoder = new Encoder(RobotMap.DIO.WHEEL_ENCODER_A, RobotMap.DIO.WHEEL_ENCODER_B);

    m_matcher.addColorMatch(Constants.COLOR_BLUE);
    m_matcher.addColorMatch(Constants.COLOR_GREEN);
    m_matcher.addColorMatch(Constants.COLOR_RED);
    m_matcher.addColorMatch(Constants.COLOR_YELLOW);
  }

  /**
   * Moves the color wheel
   *
   * @param speed the speed of the wheel motor
   */

  public void set(double speed) {
    m_wheel.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stops the wheel motor
   */
  public void stop() {
    set(0.0);
  }

  public double getDistance() {
    return m_encoder.getDistance();
  }

  public void resetEncoder() {
    m_encoder.reset();
  }

  public Color getDetectedColor() {
    return m_sensor.getColor();
  }

  public ColorMatchResult getClosestMatch() {
    return m_matcher.matchClosestColor(getDetectedColor());
  }
}
