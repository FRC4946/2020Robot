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
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoScript;
import frc.robot.commands.auto.DriveAndShoot;
import frc.robot.commands.auto.MiddlePickupAndShoot;
import frc.robot.commands.auto.PIDDrive;
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

  private Preferences m_prefs;
  private SendableChooser<AutoScript> m_autoScript;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_prefs = Preferences.getInstance();
    m_autoScript = new SendableChooser<>();
    m_autoScript.setDefaultOption("Disabled", AutoScript.DISABLED);
    m_autoScript.addOption("Drive Forwards", AutoScript.DRIVE_FORWARDS);
    m_autoScript.addOption("Drive And Shoot", AutoScript.DRIVE_AND_SHOOT);
    m_autoScript.addOption("Middle - Pickup and shoot", AutoScript.MIDDLE_PICKUP_AND_SHOOT);

    SmartDashboard.putData(m_autoScript);

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

    JoystickButton intakeButton = new JoystickButton(m_driveJoystick, RobotMap.JOYSTICK_BUTTON.INTAKE);

    JoystickButton driverShootButton = new JoystickButton(m_driveJoystick, RobotMap.JOYSTICK_BUTTON.DRIVER_SHOOT);

    JoystickButton emergencyShootButton = new JoystickButton(m_driveJoystick, RobotMap.JOYSTICK_BUTTON.EMERGENCY_SHOOT);

    JoystickButton setLimelightButton = new JoystickButton(m_operatorJoystick, RobotMap.JOYSTICK_BUTTON.USE_LIMELIGHT);

    JoystickButton spinUpButton = new JoystickButton(m_operatorJoystick, RobotMap.JOYSTICK_BUTTON.OPERATOR_SPIN_UP);

    JoystickButton extendControlPanelButton = new JoystickButton(m_operatorJoystick,
        RobotMap.JOYSTICK_BUTTON.EXTEND_CONTROL_PANEL);

    JoystickButton preset1 = new JoystickButton(m_operatorJoystick, RobotMap.JOYSTICK_BUTTON.PRESET_1);

    JoystickButton manualMode = new JoystickButton(m_operatorJoystick, RobotMap.JOYSTICK_BUTTON.MANUAL_MODE);

    JoystickButton resetPotButton = new JoystickButton(m_operatorJoystick, RobotMap.JOYSTICK_BUTTON.HOOD_BUTTON);

    JoystickButton revolverButton = new JoystickButton(m_driveJoystick, RobotMap.JOYSTICK_BUTTON.REVOLVER);

    JoystickButton manualUnjamButton = new JoystickButton(m_driveJoystick, RobotMap.JOYSTICK_BUTTON.MANUAL_UNJAM);

    // #region Operator

    // MANUAL MODE

    manualMode.whileHeld(new ManualHood(() -> {
      return Utilities.deadzone(m_operatorJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.HOOD));
    }, m_hood));

    manualMode.whileHeld(new ManualTurret(() -> {
      return Utilities.deadzone(m_operatorJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.TURRET));
    }, m_turret));

    manualMode.whileHeld(new ManualShooter(() -> {
      if (m_operatorJoystick.getPOV() == 0 || m_operatorJoystick.getPOV() == 45 || m_operatorJoystick.getPOV() == 315) {
        return (m_shooter.getSetpoint() + 50);
      } else if (m_operatorJoystick.getPOV() == 180 || m_operatorJoystick.getPOV() == 135
          || m_operatorJoystick.getPOV() == 225) {
        return (m_shooter.getSetpoint() - 50);
      }
      return m_shooter.getSetpoint();
    }, m_shooter));

    resetPotButton.and(manualMode).whenActive(new InstantCommand(() -> {
      m_hood.resetPot();
    }));

    spinUpButton.and(manualMode).whileActiveContinuous(new RunCommand(() -> {
      if (!m_shooter.isEnabled())
        m_shooter.enable();
      m_shooter.setKey(m_shooter.atSetpoint());
    }, m_shooter), false);

    preset1.and(manualMode).whileActiveContinuous(new RunCommand(() -> {
      m_turret.setSetpoint(Constants.Turret.HOME_ANGLE);
      m_shooter.setSetpoint(Constants.Shooter.PRESET_1_SPEED);
      m_hood.setSetpoint(Constants.Hood.PRESET_1_ANGLE);
      if (!m_hood.isEnabled())
        m_hood.enable();
      if (!m_shooter.isEnabled())
        m_shooter.enable();
      m_shooter.setKey(m_shooter.atSetpoint() && m_hood.atSetpoint());
    }, m_shooter, m_hood, m_turret), false);

    // STANDARD OPERATION

    setLimelightButton.and(manualMode.negate())
        .whileActiveOnce(new SetShooterWithLimelight(m_shooter, m_turret, m_hood, m_limelight, m_driveJoystick, () -> {
          return Utilities.deadzone(m_operatorJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.HOOD));
        }), false);

    extendControlPanelButton.and(manualMode.negate()).whenActive(new InstantCommand(() -> {
      m_controlPanel.setExtended(true);
    }, m_controlPanel)).whenInactive(new InstantCommand(() -> {
      m_controlPanel.setExtended(false);
    }, m_controlPanel));

    // #endregion

    // #region driver

    manualUnjamButton.whileHeld(new RunCommand(() -> {
      m_revolver.set(Constants.Revolver.BACKWARDS_SPEED);
      m_feedWheel.set(-0.6);
    }, m_revolver, m_feedWheel));

    driverShootButton.whileActiveOnce(new Shoot(m_revolver, m_shooter, m_feedWheel), false);

    emergencyShootButton.whileActiveOnce(new RunCommand(() -> {
      m_revolver.set(Constants.Revolver.FORWARDS_SPEED);
      m_feedWheel.set(0.6);
    }, m_revolver, m_feedWheel));

    climbButton.toggleWhenPressed(new Climb(() -> Utilities
        .deadzone(Math.pow(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.CLIMB_1), 2)
            + Math.pow(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.CLIMB_2), 2))
        * Constants.Climber.MAX_PERCENT_OUTPUT, m_climber, m_intake, m_shooter), false);

    intakeButton.whenPressed(new InstantCommand(() -> {
      if (m_intake.isExtended()) {
        m_intake.setExtended(false);
        m_intake.stop();
        m_revolver.stop();
      } else {
        m_intake.setExtended(true);
      }
    }));

    revolverButton.whenHeld(new RunCommand(() -> {
      m_revolver.set(4 * Constants.Revolver.FORWARDS_SPEED);
    }, m_revolver));

    // #endregion

    // #region Default Commands

    m_driveTrain.setDefaultCommand(new RunCommand(() -> {
      m_driveTrain.arcadeDrive(-m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.DRIVE),
          -m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.TURN));
    }, m_driveTrain));

    m_intake.setDefaultCommand(new RunCommand(() -> {
      if (m_intake.isExtended()) {
        m_intake.set(Utilities.deadzone(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.INTAKE)
            - m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.OUTTAKE)));
      } else {
        m_intake.stop();
      }
    }, m_intake));

    m_revolver.setDefaultCommand(new RunCommand(() -> {
      if (m_intake.isExtended() && Math.abs(m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.INTAKE)
          - m_driveJoystick.getRawAxis(RobotMap.JOYSTICK_AXIS.OUTTAKE)) > Constants.DEFAULT_DEADZONE) {
        m_revolver.set(2 * Constants.Revolver.FORWARDS_SPEED);
      } else {
        m_revolver.stop();
      }
    }, m_revolver));

    m_shooter.setDefaultCommand(new RunCommand(() -> {
      if (m_shooter.isEnabled()) {
        m_shooter.disable();
      }
      m_shooter.set(Constants.Shooter.IDLE_SPEED * Constants.Shooter.VELOCITY_FF); // Run at resting speed
    }, m_shooter));

    m_hood.setDefaultCommand(new RunCommand(() -> {
      if (!m_hood.isEnabled()) {
        m_hood.enable();
      }
      m_hood.setSetpoint(Constants.Hood.MIN_ANGLE);
    }, m_hood));

    m_turret.setDefaultCommand(new RunCommand(() -> {
      //m_turret.holdPosition();
      m_turret.setSetpoint(Constants.Turret.HOME_ANGLE);
    }, m_turret));

    m_controlPanel.setDefaultCommand(new RunCommand(() -> {
      m_controlPanel.stop();
    }, m_controlPanel));

    m_feedWheel.setDefaultCommand(new RunCommand(() -> {
      m_feedWheel.stop();
    }, m_feedWheel));

    // #endregion
  }

  /**
   * Resets sensors, should be called in Robot.robotInit
   */
  public void robotInit() {
    m_prefs.putDouble("drive/startX", m_prefs.getDouble("drive/startX", 0.0));
    m_prefs.putDouble("drive/startY", m_prefs.getDouble("drive/startY", 0.0));
    m_prefs.putDouble("drive/startAngle", m_prefs.getDouble("drive/startAngle", 0.0));

    m_driveTrain.resetDriveTrain(m_prefs.getDouble("drive/startX", 0.0), m_prefs.getDouble("drive/startY", 0.0),
        m_prefs.getDouble("drive/startAngle", 0.0));
    m_hood.resetPot();
  }

  /**
   * Resets sensors and schedules the autonomous command.
   */
  public void setupAuto() {
    robotInit();

    AutoScript script = m_autoScript.getSelected();

    switch (script) {
    case DRIVE_AND_SHOOT:
      m_autonomousCommand = new DriveAndShoot(m_driveTrain, m_feedWheel, m_hood, m_limelight, m_revolver, m_shooter,
          m_turret);
      break;
    case MIDDLE_PICKUP_AND_SHOOT:
      m_autonomousCommand = new MiddlePickupAndShoot(m_driveTrain, m_feedWheel, m_hood, m_intake, m_limelight,
          m_revolver, m_shooter, m_turret);
      break;
    case DRIVE_FORWARDS:
      /*m_autonomousCommand = new RunCommand(() -> m_driveTrain.arcadeDrive(0.5, 0.0), m_driveTrain).withTimeout(2)
          .andThen(new RunCommand(() -> {
            m_driveTrain.arcadeDrive(-0.1, 0.0);
          }, m_driveTrain).withTimeout(0.1));*/
      m_autonomousCommand = new PIDDrive(1.0, m_driveTrain);
      break;
    case DISABLED:
    default:
      m_autonomousCommand = null;
    }

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
