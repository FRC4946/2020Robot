/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.hood.ManualHood;
import frc.robot.commands.revolver.Shoot;
import frc.robot.commands.shooter.ManualShooter;
import frc.robot.commands.shooter.SetShooterWithLimelight;
import frc.robot.commands.turret.ManualTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Revolver;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.util.Utilities;

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

  private final Climber m_climber;
  private final ControlPanel m_controlPanel;
  private final DriveTrain m_driveTrain;
  private final FeedWheel m_feedWheel;
  private final Hood m_hood;
  private final Intake m_intake;
  private final Limelight m_limelight;
  private final Revolver m_revolver;
  private final Shooter m_shooter;
  private final Turret m_turret;

  private Command m_autonomousCommand = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveJoystick = new Joystick(RobotMap.JOYSTICK.DRIVER);
    m_operatorJoystick = new Joystick(RobotMap.JOYSTICK.OPERATOR);

    m_climber = new Climber();
    m_controlPanel = new ControlPanel();
    m_driveTrain = new DriveTrain();
    m_feedWheel = new FeedWheel();
    m_hood = new Hood();
    m_intake = new Intake();
    m_limelight = new Limelight();
    m_revolver = new Revolver();
    m_shooter = new Shooter();
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
    JoystickButton operatorShootButton = new JoystickButton(m_operatorJoystick, RobotMap.JOYSTICK_BUTTON.USE_LIMELIGHT);
    JoystickButton spinUp = new JoystickButton(m_operatorJoystick, RobotMap.JOYSTICK_BUTTON.OPERATOR_SPIN_UP);
    JoystickButton extendControlPanel = new JoystickButton(m_operatorJoystick,
        RobotMap.JOYSTICK_BUTTON.EXTEND_CONTROL_PANEL);
    JoystickButton shiftGear = new JoystickButton(m_driveJoystick, RobotMap.JOYSTICK_BUTTON.SHIFT_GEAR);
    JoystickButton preset1 = new JoystickButton(m_operatorJoystick, RobotMap.JOYSTICK_BUTTON.PRESET_1);

    shiftGear.whenPressed(new InstantCommand(() -> {
      m_driveTrain.setHighGear(!m_driveTrain.isHighGear());
    }));

    preset1.whileHeld(new RunCommand(() -> {
      m_shooter.setSetpoint(Constants.Shooter.PRESET_1_SPEED);
      m_hood.setSetpoint(Constants.Hood.PRESET_1_ANGLE);
      if (!m_shooter.isEnabled())
        m_shooter.enable();
      if (!m_hood.isEnabled())
        m_hood.enable();
      m_shooter.setKey(m_shooter.atSetpoint() && m_hood.atSetpoint());
    }, m_shooter, m_hood));

    operatorShootButton
        .whenHeld(new SetShooterWithLimelight(m_driveJoystick, m_shooter, m_turret, m_hood, m_limelight));
    spinUp.whileHeld(new RunCommand(() -> {
      if (!m_shooter.isEnabled())
        m_shooter.enable();
      m_shooter.setKey(m_shooter.atSetpoint());
    }, m_shooter));

    driverShootButton.whileActiveOnce(new Shoot(m_revolver, m_shooter, m_feedWheel), false);

    extendControlPanel.whenPressed(new InstantCommand(() -> {
      m_controlPanel.setExtended(true);
    }, m_controlPanel));
    extendControlPanel.whenReleased(new InstantCommand(() -> {
      m_controlPanel.setExtended(false);
    }, m_controlPanel));

    climbButton.toggleWhenPressed(new Climb(() -> Utilities
        .deadzone(Math.pow(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.CLIMB_1), 2)
            + Math.pow(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.CLIMB_2), 2))
        * Constants.Climber.MAX_PERCENT_OUTPUT, m_climber, m_intake), false);

    intake.whenHeld(new RunCommand(() -> {
      m_intake.set(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.INTAKE)
          - m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.OUTTAKE));
    }, m_intake));

    intake.whileHeld(new RunCommand(() -> {
      if (Math.abs(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.INTAKE)
          - m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.OUTTAKE)) > Constants.DEFAULT_DEADZONE) {
        m_revolver.set(Constants.Revolver.FORWARDS_SPEED);
      } else {
        m_revolver.stop();
      }
    }, m_revolver));

    intake.whenPressed(new InstantCommand(() -> {
      m_intake.setExtended(true);
    }));

    intake.whenReleased(new InstantCommand(() -> {
      m_intake.setExtended(false);
      m_intake.stop();
      m_revolver.stop();
    }));

    // Default Commands

    m_driveTrain.setDefaultCommand(new RunCommand(() -> {
      m_driveTrain.arcadeDrive(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.DRIVE),
          m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.TURN));
    }, m_driveTrain));

    m_revolver.setDefaultCommand(new RunCommand(() -> {
      m_revolver.stop();
    }, m_revolver));

    m_hood.setDefaultCommand(new ManualHood(() -> {
      return Utilities.deadzone(m_operatorJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.HOOD));
    }, m_hood));

    m_turret.setDefaultCommand(new ManualTurret(() -> {
      return Utilities.deadzone(m_operatorJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.TURRET));
    }, m_turret));

    m_shooter.setDefaultCommand(new ManualShooter(() -> {
      if (m_operatorJoystick.getPOV() == 0 || m_operatorJoystick.getPOV() == 45 || m_operatorJoystick.getPOV() == 315) {
        return (m_shooter.getSetpoint() + 50);
      } else if (m_operatorJoystick.getPOV() == 180 || m_operatorJoystick.getPOV() == 135
          || m_operatorJoystick.getPOV() == 225) {
        return (m_shooter.getSetpoint() - 50);
      }
      return m_shooter.getSetpoint();
    }, m_shooter));
  }

  /**
   * Resets sensors, should be called in Robot.robotInit
   */
  public void robotInit() {
    m_driveTrain.resetDriveTrain();
    m_hood.resetPot();
  }

  /**
   * Resets sensors and schedules the autonomous command.
   */
  public void setupAuto() {
    robotInit();

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

  public static Color getFMSColor() {
    switch (DriverStation.getInstance().getGameSpecificMessage()) {
      case "B":
        return Constants.ControlPanel.COLOR_BLUE;
      case "G":
        return Constants.ControlPanel.COLOR_GREEN;
      case "R":
        return Constants.ControlPanel.COLOR_RED;
      case "Y":
        return Constants.ControlPanel.COLOR_YELLOW;
      default:
        return null;
    }
  }
}
