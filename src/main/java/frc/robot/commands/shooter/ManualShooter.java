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
  private final DoubleSupplier m_POVSupplier;

  /**
   * Sets the setpoint shooter speed manually, and runs the shooter at its resting
   * speed
   *
   * @param joystick the joystick to use to set the setpoint
   * @param shooter  the shooter to use for this command
   */
  public ManualShooter(DoubleSupplier POVSupplier, Shooter shooter) {
    m_shooter = shooter;
    m_POVSupplier = POVSupplier;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.disable();
  }

  @Override
  public void execute() {
    m_shooter.set(Constants.Shooter.IDLE_SPEED * Constants.Shooter.VELOCITY_FF); // Run at resting speed
    if (m_POVSupplier.getAsDouble() == 0 || m_POVSupplier.getAsDouble() == 45 || m_POVSupplier.getAsDouble() == 315) {
      m_shooter.setSetpoint(m_shooter.getSetpoint() + 50);
    } else if (m_POVSupplier.getAsDouble() == 180 || m_POVSupplier.getAsDouble() == 135
        || m_POVSupplier.getAsDouble() == 225) {
      m_shooter.setSetpoint(m_shooter.getSetpoint() - 50);
    }
  }
}
