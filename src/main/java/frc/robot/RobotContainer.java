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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.hood.ManualHood;
import frc.robot.commands.revolver.RunRevolver;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.turret.ManualTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Revolver;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick m_driveJoystick;
  private final Joystick m_operatorJoystick;

  private final DriveTrain m_driveTrain;
  private final Shooter m_shooter;
  private final Climber m_climber;
  private final Limelight m_limelight;
  private final Hood m_hood;
  private final Revolver m_revolver;
  private final Intake m_intake;
  private final Turret m_turret;

  private Command m_autonomousCommand = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveJoystick = new Joystick(RobotMap.JOYSTICK.DRIVE_JOYSTICK);
    m_operatorJoystick = new Joystick(RobotMap.JOYSTICK.OPERATOR_JOYSTICK);

    m_driveTrain = new DriveTrain();
    m_shooter = new Shooter();
    m_climber = new Climber();
    m_limelight = new Limelight();
    m_revolver = new Revolver();
    m_hood = new Hood();
    m_intake = new Intake();
    m_turret = new Turret();
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Buttons

    JoystickButton climbButton = new JoystickButton(m_driveJoystick, RobotMap.JOYSTICK_BUTTON.CLIMB);

    JoystickButton intake = new JoystickButton(m_driveJoystick, RobotMap.JOYSTICK_BUTTON.INTAKE);

    JoystickButton driverShootButton = new JoystickButton(m_driveJoystick, RobotMap.JOYSTICK_BUTTON.DRIVER_SHOOT);
    JoystickButton operatorShootButton = new JoystickButton(m_operatorJoystick,
        RobotMap.JOYSTICK_BUTTON.OPERATOR_SHOOT);

    driverShootButton.and(operatorShootButton)
        .whileActiveOnce(new Shoot(Constants.SHOOT_SPEED, m_hood.getSetpoint(), m_shooter, m_hood, m_revolver), false);

    climbButton.toggleWhenPressed(
        new Climb(m_driveJoystick, RobotMap.JOYSTICK_AXIS.CLIMB_1, RobotMap.JOYSTICK_AXIS.CLIMB_2, m_climber));

    intake.whenHeld(new RunRevolver(Constants.REVOLVER_DRUM_FORWARDS_SPEED, 0.0, m_revolver));

    // Default Commands

    m_driveTrain.setDefaultCommand(new RunCommand(() -> {
      m_driveTrain.arcadeDrive(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.DRIVE),
          m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.TURN));
    }, m_driveTrain));

    m_intake.setDefaultCommand(new RunCommand(() -> {
      if (intake.get()) {
        m_intake.setExtended(true);
        m_intake.set(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.INTAKE)
            - m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.OUTTAKE));
      } else {
        m_intake.set(0.0);
        m_intake.setExtended(false);
      }
    }, m_intake));

    m_revolver.setDefaultCommand(new RunCommand(() -> {
      m_revolver.stop();
    }, m_revolver));

    m_hood.setDefaultCommand(new ManualHood(() -> {
      if (m_operatorJoystick.getPOV() == 0 || m_operatorJoystick.getPOV() == 45 || m_operatorJoystick.getPOV() == 315) {
        return 0.6;
      } else if (m_operatorJoystick.getPOV() == 180 || m_operatorJoystick.getPOV() == 135 || m_operatorJoystick.getPOV() == 225) {
        return -0.6;
      } 
      return 0.0;
    }, m_hood));

    m_turret.setDefaultCommand(new ManualTurret(() -> {
      if (m_operatorJoystick.getPOV() == 90 || m_operatorJoystick.getPOV() == 45 || m_operatorJoystick.getPOV() == 135) {
        return 0.3;
      } else if (m_operatorJoystick.getPOV() == 270 || m_operatorJoystick.getPOV() == 225 || m_operatorJoystick.getPOV() == 315) {
        return -0.3;
      } 
      return 0.0;
    }, m_turret));
  }

  /**
   * Resets encoders, gyro, any other sensors necessary for auto, etc...
   * Also schedules autonomous command
   */
  public void setupAuto() {
    m_driveTrain.resetDriveTrain();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * Cancels the current autonomous command
   */
  public void cancelAuto() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
}
