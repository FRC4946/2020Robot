/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ControlPanel extends SubsystemBase {

  private final ColorSensorV3 m_sensor;
  private final TalonSRX m_wheel;
  private final ColorMatch m_matcher;

  public ControlPanel() {
    m_sensor = new ColorSensorV3(Port.kOnboard);
    m_wheel = new TalonSRX(RobotMap.CAN.TALONSRX_CONTROL_PANEL);
    m_matcher = new ColorMatch();

    m_wheel.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    m_matcher.addColorMatch(Constants.ControlPanel.COLOR_BLUE);
    m_matcher.addColorMatch(Constants.ControlPanel.COLOR_GREEN);
    m_matcher.addColorMatch(Constants.ControlPanel.COLOR_RED);
    m_matcher.addColorMatch(Constants.ControlPanel.COLOR_YELLOW);
  }

  /**
   * Moves the color wheel.
   *
   * @param speed the speed of the wheel motor
   */
  public void set(double speed) {
    m_wheel.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stops the wheel motor.
   */
  public void stop() {
    set(0.0);
  }

  /**
   * Gets the encoder position, in degrees.
   */
  public double getDistance() {
    return m_wheel.getSelectedSensorPosition() * Constants.ControlPanel.ENCODER_DEGREES_PER_TICK;
  }

  /**
   * Resets the encoder position to 0.
   */
  public void resetEncoder() {
    m_wheel.setSelectedSensorPosition(0);
  }

  /**
   * Gets the closest matching color detected by the color sensor.
   */
  public ColorMatchResult getCurrentColor() {
    return m_matcher.matchClosestColor(m_sensor.getColor());
  }
}
