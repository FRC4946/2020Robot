/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ShootAtSpeed;
import frc.robot.commands.climber.Climb;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  Joystick m_driveJoystick;
  Joystick m_operatorJoystick;

  DriveTrain m_driveTrain;
  Shooter m_shooter;
  Climber m_climber;
  ConveyorBelt m_conveyor;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_driveTrain = new DriveTrain();
    m_shooter = new Shooter();
    m_climber = new Climber();
    m_conveyor = new ConveyorBelt();

    configureButtonBindings();

    m_driveTrain.setDefaultCommand(new RunCommand(() -> {
      m_driveTrain.arcadeDrive(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.DRIVE_AXIS),
          m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.TURN_AXIS));
    }, m_driveTrain));

    m_shooter.setDefaultCommand(new ShootAtSpeed(m_shooter));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driveJoystick = new Joystick(RobotMap.JOYSTICK.DRIVE_JOYSTICK);
    m_operatorJoystick = new Joystick(RobotMap.JOYSTICK.OPERATOR_JOYSTICK);

    JoystickButton climbButton = new JoystickButton(m_driveJoystick, RobotMap.JOYSTICK_BUTTON.CLIMB_BUTTON);
    Climb climbCommand = new Climb(m_driveJoystick, RobotMap.JOYSTICK_AXIS.CLIMB_AXIS_1,
        RobotMap.JOYSTICK_AXIS.CLIMB_AXIS_2, m_climber);
    climbButton.whenPressed(new InstantCommand(() -> {
      if (climbCommand.isScheduled()) {
        climbCommand.cancel();
      } else {
        climbCommand.schedule();
      }
    }));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
