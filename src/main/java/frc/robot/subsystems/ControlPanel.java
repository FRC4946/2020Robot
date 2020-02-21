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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ControlPanel extends SubsystemBase {

  private final ColorSensorV3 m_sensor;
  private final TalonSRX m_wheel;
  private final ColorMatch m_matcher;
  private final Encoder m_encoder;

  public ControlPanel() {
    m_sensor = new ColorSensorV3(Port.kOnboard);
    m_wheel = new TalonSRX(RobotMap.CAN.CONTROL_PANEL_TALONSRX);
    m_matcher = new ColorMatch();
    m_encoder = new Encoder(RobotMap.DIO.WHEEL_ENCODER_A, RobotMap.DIO.WHEEL_ENCODER_B);

    m_encoder.setDistancePerPulse(Constants.CONTROL_PANEL_ENCODER_DEGREES_PER_TICK);

    m_matcher.addColorMatch(Constants.COLOR_BLUE);
    m_matcher.addColorMatch(Constants.COLOR_GREEN);
    m_matcher.addColorMatch(Constants.COLOR_RED);
    m_matcher.addColorMatch(Constants.COLOR_YELLOW);
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
    return m_encoder.getDistance();
  }

  /**
   * Resets the encoder position to 0.
   */
  public void resetEncoder() {
    m_encoder.reset();
  }

  /**
   * Gets the closest matching color detected by the color sensor.
   */
  public ColorMatchResult getCurrentColor() {
    return m_matcher.matchClosestColor(m_sensor.getColor());
  }
}
