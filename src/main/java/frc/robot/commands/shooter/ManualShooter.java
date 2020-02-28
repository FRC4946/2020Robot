/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ManualShooter extends CommandBase {

  private final Shooter m_shooter;
  private final DoubleSupplier m_speedSupplier;

  /**
   * Sets the setpoint shooter speed manually, and runs the shooter at its resting
   * speed
   *
   * @param speedSuplier the speed to run the shooter
   * @param shooter      the shooter to use for this command
   */
  public ManualShooter(DoubleSupplier speedSupplier, Shooter shooter) {
    m_shooter = shooter;
    m_speedSupplier = speedSupplier;
    addRequirements(m_shooter);
  }

  @Override
  public void execute() {
    if (m_shooter.isEnabled())
      m_shooter.disable();
    m_shooter.set(Constants.Shooter.IDLE_SPEED * Constants.Shooter.VELOCITY_FF); // Run at resting speed
    m_shooter.setSetpoint(m_speedSupplier.getAsDouble());
  }
}
