// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.Actions.Align.IntakeAlign;
import frc.robot.commands.Actions.Align.ShooterXAlign;
import frc.robot.commands.Actions.Movement.MoveToPosition;
import frc.robot.commands.Actions.Movement.RunBasicTrajectory;
import frc.robot.commands.Continuous.DriveCommand;
import frc.robot.commands.Groups.Auto_1B;
import frc.robot.commands.Groups.Auto_2B_1;
import frc.robot.commands.Groups.Auto_2B_2;
import frc.robot.commands.Groups.Auto_2B_3;
import frc.robot.other.FilteredController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  private final static HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
  private final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final static LifterSubsystem m_lifterSubsystem = new LifterSubsystem();
  private final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final static XboxController m_controller1 = new XboxController(0);
  public static final FilteredController filteredController1 = new FilteredController(m_controller1);

  private final static XboxController m_controller2 = new XboxController(1);
  public static final FilteredController filteredController2 = new FilteredController(m_controller2);
  private static SendableChooser m_chooser;
  public static final DriveCommand m_driveCommand = new DriveCommand(
      m_driveSubsystem,
      () -> -modifyAxis(filteredController1.getYLeft(.2)) * -DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND
          * Constants.outputs.strafe,
      () -> -modifyAxis(filteredController1.getXLeft(.2)) * -DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND
          * Constants.outputs.strafe,
      () -> -modifyAxis(filteredController1.getXRight(.2)) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
          * Constants.outputs.turnRate);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  @SuppressWarnings("unchecked")
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Nothing", null);
    m_chooser.addOption("2Ball1", new Auto_2B_1());
    m_chooser.addOption("2Ball2", new Auto_2B_2());
    m_chooser.addOption("2Ball3", new Auto_2B_3());
    m_chooser.addOption("1Ball", new Auto_1B());

    SmartDashboard.putData(m_chooser);
    m_driveSubsystem.setDefaultCommand(m_driveCommand);
    Logger.configureLoggingAndConfig(this, false);
    Logger.setCycleWarningsEnabled(false);
    recalibrateGyroscope();
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
    //new Button(m_controller1::getAButton).whenPressed(new ResetGyro(m_driveSubsystem));

    new Button(m_controller1::getAButton).whenHeld(new IntakeAlign(m_driveSubsystem));
    new Button(m_controller1::getXButton).whenHeld(new ShooterXAlign(m_driveSubsystem));
    new Button(m_controller1::getYButton).whenPressed(new MoveToPosition(m_driveSubsystem, new Pose2d(1, 0, new Rotation2d())));
    new Button(m_controller1::getBButton).whenPressed(new RunBasicTrajectory(m_driveSubsystem, "New Path"));

    /*
    // DRIVER 1
    new Button(m_controller1::getAButton).toggleWhenPressed(new Defend(m_driveSubsystem));
    new Button(m_controller1::getYButton).toggleWhenPressed(new ClimbSequence(m_climberSubsystem));
    new Button(m_controller1::getRightBumper).whenHeld(new IntakeAlign(m_driveSubsystem));
    //new Button(m_controller1::getStartButton).whenPressed(new ForceReleaseLower(m_lifterSubsystem, m_intakeSubsystem));
    //new Button(m_controller1::getBackButtonPressed).whenPressed(new ForceReleaseUpper(m_lifterSubsystem, m_shooterSubsystem,m_hoodSubsystem));
    new Button(filteredController1::getRightTriggerActive).whileHeld(new IntakeSequence(m_intakeSubsystem,m_lifterSubsystem));

    
    // DRIVER 2 
    new Button(filteredController2::getRightTriggerActive).whileHeld(new Spool(m_shooterSubsystem, Constants.subsystems.shooter.targetRPM));
    new Button(m_controller2::getRightBumper).whenHeld(new Shoot(m_lifterSubsystem));
    //new Button(m_controller2::getStartButton).whenPressed(new ForceReleaseLower(m_lifterSubsystem, m_intakeSubsystem));
    //new Button(m_controller2::getBackButtonPressed).whenPressed(new ForceReleaseUpper(m_lifterSubsystem, m_shooterSubsystem,m_hoodSubsystem));
    new Button(m_controller2::getLeftBumper).whenHeld(new ShooterAlignSequence(m_driveSubsystem, m_hoodSubsystem));
    new Button(m_controller2::getLeftStickButton).toggleWhenPressed(new ToggleManualHood(m_hoodSubsystem));
   */ 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return (Command) m_chooser.getSelected();
  }

  public void resetOdometry() {
    m_driveSubsystem.resetOdometry();
  }

  public void recalibrateGyroscope() {
    m_driveSubsystem.zeroGyroscope();
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

  public static HoodSubsystem getHoodSubsystem() {
    return m_hoodSubsystem;
  }

  public static IntakeSubsystem getIntakeSubsystem() {
    return m_intakeSubsystem;
  }

  public static LifterSubsystem getLifterSubsystem() {
    return m_lifterSubsystem;
  }

  public static ShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }

  public void publishPosition() {
    Logger.updateEntries();
  }
  public static FilteredController getDriver1(){return filteredController1;}
  public static FilteredController getDriver2(){return filteredController2;}

}
