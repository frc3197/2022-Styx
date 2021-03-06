// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.Align.IntakeAlign;
import frc.robot.commands.Auto.Lookups.AutoLookup;
import frc.robot.commands.Climber.RotateClimber;
import frc.robot.commands.Climber.SpoolClimber;
import frc.robot.commands.Drivetrain.Defend;
import frc.robot.commands.Drivetrain.DriveCommand;
import frc.robot.commands.Drivetrain.DriveCommand.DriveType;
import frc.robot.commands.Drivetrain.ResetGyro;
import frc.robot.commands.Groups.IntakeSequence;
import frc.robot.commands.Groups.ReplaceCargo;
import frc.robot.commands.Groups.ReverseBallPath;
import frc.robot.commands.Groups.ShootSequence;
import frc.robot.commands.Groups.ShooterAlignSequence;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Lifter.IntakeRumble;
import frc.robot.commands.Lifter.SpitBoth;
import frc.robot.commands.Lifter.SpitLower.CargoReleaseSpeed;
import frc.robot.commands.Shooter.RangeLookup;
import frc.robot.commands.Shooter.Spool;
import frc.robot.other.Toggles.ResetHood;
import frc.robot.other.Toggles.ToggleBrakeMode;
import frc.robot.other.Toggles.ToggleDriveSpeeds;
import frc.robot.other.Toggles.ToggleFieldRelative;
import frc.robot.other.Toggles.ToggleManualHood;
import frc.robot.other.extra_libraries.FilteredController;
import frc.robot.subsystems.Climber.ClimberArm;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeArm;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.LifterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import io.github.oblarg.oblog.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final static DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final static ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final static ClimberArm m_climberArmSubsystem = new ClimberArm();
  private final static HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
  private final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final static IntakeArm m_intakeArmSubsystem = new IntakeArm();
  private final static LifterSubsystem m_lifterSubsystem = new LifterSubsystem();
  private final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final static XboxController m_controller1 = new XboxController(0);
  public static final FilteredController filteredController1 = new FilteredController(m_controller1);
  private final static XboxController m_controller2 = new XboxController(1);
  public static final FilteredController filteredController2 = new FilteredController(m_controller2);

  private static SlewRateLimiter[] slimiters = DriveCommand.getLimiterArray(DriveType.NORMAL);

  @SuppressWarnings("rawtypes")
  private static SendableChooser m_autoChooser;
  @SuppressWarnings("rawtypes")
  private static SendableChooser m_allianceChooser;

  public static final DriveCommand m_driveCommand = new DriveCommand(
      m_driveSubsystem,
      () -> -modifyAxis(filteredController1.getYLeft(.2)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND
          * Constants.outputs.strafe,
      () -> -modifyAxis(filteredController1.getXLeft(.2)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND
          * Constants.outputs.strafe,
      () -> -modifyAxis(filteredController1.getXRight(.2)) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
          * Constants.outputs.turnRate);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  @SuppressWarnings("unchecked")
  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_autoChooser = new SendableChooser<>();
    m_autoChooser.setDefaultOption("Nothing", null);

    m_autoChooser.addOption("2BL1", AutoLookup.getAuto("2BL1"));
    m_autoChooser.addOption("2BL1F1", AutoLookup.getAuto("2BL1F1"));
    m_autoChooser.addOption("2BL1F2", AutoLookup.getAuto("2BL1F2"));
    
    m_autoChooser.addOption("2BL2", AutoLookup.getAuto("2BL2"));
    m_autoChooser.addOption("2BL2F1", AutoLookup.getAuto("2BL2F1"));
    m_autoChooser.addOption("2BL2F2", AutoLookup.getAuto("2BL2F2"));
    
    m_autoChooser.addOption("2BL3", AutoLookup.getAuto("2BL3"));
    m_autoChooser.addOption("2BL3F3", AutoLookup.getAuto("2BL3F3"));
    m_autoChooser.addOption("2BL3F4", AutoLookup.getAuto("2BL3F4"));
    
    m_autoChooser.addOption("2BL4", AutoLookup.getAuto("2BL4"));
    m_autoChooser.addOption("2BL4F3", AutoLookup.getAuto("2BL4F3"));
    m_autoChooser.addOption("2BL4F4", AutoLookup.getAuto("2BL4F4"));

    m_autoChooser.addOption("3BL4", AutoLookup.getAuto("3BL4"));
    m_autoChooser.addOption("3BL4F", AutoLookup.getAuto("3BL4F"));

    m_autoChooser.addOption("4BL3", AutoLookup.getAuto("4BL3"));
    m_autoChooser.addOption("4BL3F", AutoLookup.getAuto("4BL3F"));

    m_autoChooser.addOption("4BL4", AutoLookup.getAuto("4BL4"));
    m_autoChooser.addOption("4BL4F", AutoLookup.getAuto("4BL4F"));

    m_autoChooser.addOption("5BL4", AutoLookup.getAuto("5BL4"));
    m_autoChooser.addOption("5BL4F", AutoLookup.getAuto("5BL4F"));

    m_autoChooser.addOption("2BFEN",AutoLookup.getAuto("2BFEN"));
    m_autoChooser.addOption("2BFEN_ALT",AutoLookup.getAuto("2BFEN_ALT"));
    m_autoChooser.addOption("2E_STE.4",AutoLookup.getAuto("2E_STE.4"));
    m_autoChooser.addOption("1E.2.1",AutoLookup.getAuto("1E.2.1"));
    m_autoChooser.addOption("1E.4FEN", AutoLookup.getAuto("1E.4FEN"));
    SmartDashboard.putData(m_autoChooser);
    m_driveSubsystem.setDefaultCommand(m_driveCommand);
    CommandScheduler.getInstance().schedule(new IntakeRumble().perpetually());
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // DRIVER 1
    new Button(m_controller1::getAButton).whileHeld(new ReplaceCargo(CargoReleaseSpeed.FAST));
    new Button(m_controller1::getAButton).whenReleased(new RetractIntake(getIntakeArmSubsystem()));
    new Button(m_controller1::getXButton).whileHeld(new Spool(getShooterSubsystem(), 1500));
    new Button(m_controller1::getBButton).whenPressed(new RetractIntake(getIntakeArmSubsystem()));
    new Button(m_controller1::getYButton).whileHeld(new SpitBoth(getIntakeSubsystem(), getLifterSubsystem(), SpitBoth.CargoReleaseSpeed.SLOW));


    new Button(m_controller1::getStartButton).whenPressed(new ResetGyro(m_driveSubsystem));
    new Button(m_controller1::getBackButton).whenPressed(new ToggleFieldRelative());
    new Button(m_controller1::getLeftStickButton).whenPressed(new ToggleBrakeMode());
    new Button(m_controller1::getRightStickButton).whenPressed(new ToggleDriveSpeeds());

    new Button(filteredController1::getRightTriggerActive)
        .whenHeld(new IntakeSequence(m_intakeSubsystem, m_lifterSubsystem, m_intakeArmSubsystem)
            .andThen(new RetractIntake(m_intakeArmSubsystem)));
    new Button(filteredController1::getRightTriggerActive).whenReleased(new RetractIntake(m_intakeArmSubsystem));
    new Button(filteredController1::getLeftTriggerActive).whileHeld(new Defend(getDriveSubsystem()));

    new Button(m_controller1::getRightBumper).whileHeld(new IntakeAlign(getDriveSubsystem()));
    new Button(m_controller1::getLeftBumper).whenPressed(new RetractIntake(m_intakeArmSubsystem));

    // DRIVER 2
    new Button(m_controller2::getLeftBumper).whileHeld(new ShooterAlignSequence(m_driveSubsystem, m_hoodSubsystem));
    new Button(filteredController2::getRightTriggerActive).whileHeld(new Spool(getShooterSubsystem()));
    new Button(m_controller2::getRightBumper).whenHeld(new ShootSequence(m_lifterSubsystem));
    new Button(m_controller2::getStartButtonPressed).whenHeld(new ResetHood());
    new Button(m_controller2::getYButton).whenHeld(new SpoolClimber(m_climberSubsystem, "Up"));
    new Button(m_controller2::getAButton).whenHeld(new SpoolClimber(m_climberSubsystem, "Down"));
    new Button(m_controller2::getBButton).whenHeld(new ReverseBallPath().beforeStarting(new WaitCommand(.1)));
    new Button(m_controller2::getRightStickButton).toggleWhenPressed(new RotateClimber(m_climberArmSubsystem));

    new Button(m_controller2::getLeftStickButton).toggleWhenPressed(new ToggleManualHood(m_hoodSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    return (Command) m_autoChooser.getSelected();
  }

  public void resetOdometry() {
    m_driveSubsystem.resetOdometry();
  }

  public void recalibrateGyroscope() {
    m_driveSubsystem.zeroGyroscope();
  }

  @SuppressWarnings("rawtypes")
  public static SendableChooser getAllianceChooser() {
    return m_allianceChooser;
  }

  /**
   * @param value
   * @param deadband
   * @return double
   */
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * @param value
   * @return double
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public static DriveSubsystem getDriveSubsystem() {
    return m_driveSubsystem;
  }

  public static ClimberSubsystem getClimberSubsystem() {
    return m_climberSubsystem;
  }

  public static ClimberArm getClimberArmSubsystem() {
    return m_climberArmSubsystem;
  }

  public static HoodSubsystem getHoodSubsystem() {
    return m_hoodSubsystem;
  }

  public static IntakeSubsystem getIntakeSubsystem() {
    return m_intakeSubsystem;
  }

  public static IntakeArm getIntakeArmSubsystem() {
    return m_intakeArmSubsystem;
  }

  public static LifterSubsystem getLifterSubsystem() {
    return m_lifterSubsystem;
  }

  public static ShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }

  public static int getTeamColor() {
    if (m_allianceChooser.getSelected().equals("Blue")) {
      return 0;
    } else {
      return 1;
    }
  }

  public void publishPosition() {
    SmartDashboard.putBoolean("In Range", RangeLookup.convertLLYtoRange(
        NetworkTableInstance.getDefault().getTable("limelight-styx").getEntry("ty").getDouble(0)) < 180);

    Logger.updateEntries();
  }

  public static FilteredController getDriver1() {
    return filteredController1;
  }

  public static FilteredController getDriver2() {
    return filteredController2;
  }

  public static SlewRateLimiter[] getLimiters() {
    return slimiters;
  }
  public static void setLimiters(DriveType driveType) {
    slimiters = DriveCommand.getLimiterArray(driveType);
  }

}
