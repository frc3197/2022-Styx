// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.Actions.Movement.RunBasicTrajectory;
import frc.robot.commands.Toggles.Defend;
import frc.robot.commands.Toggles.ToggleFieldRelative;
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

  private final static XboxController m_controller = new XboxController(0);
  public static final FilteredController filteredController = new FilteredController(m_controller);
  public static final DriveCommand m_driveCommand = new DriveCommand(
      m_driveSubsystem,
      () -> -modifyAxis(filteredController.getYLeft(.2)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND
          * Constants.outputs.strafe,
      () -> -modifyAxis(filteredController.getXLeft(.2)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND
          * Constants.outputs.strafe,
      () -> -modifyAxis(filteredController.getXRight(.2)) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
          * Constants.outputs.turnRate);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
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
    // Back button zeros the gyroscope
    new Button(m_controller::getYButton).whenPressed(m_driveCommand);
    // new Button(m_controller::getXButton).whenPressed(new
    // MoveToPosition(m_drivetrainSubsystem, new Pose2d(0,0 , new
    // Rotation2d(Math.PI))));
    new Button(m_controller::getXButton)
        .whenPressed(new RunBasicTrajectory(m_driveSubsystem, "Test Path Correct Ver"));
    new Button(m_controller::getAButton).whileHeld(new Defend(m_driveSubsystem));
    new Button(m_controller::getBackButton)
        // No requirements because we don't need to interrupt anything
        .whenPressed(m_driveSubsystem::zeroGyroscope);

    new Button(m_controller::getStartButton).whenPressed(new ToggleFieldRelative());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  public void resetOdometry() {
    m_driveSubsystem.resetOdometry();
  }

  public void recalibrateGyroscope() {
    m_driveSubsystem.getGyroscopeObj().calibrate();
    m_driveSubsystem.getGyroscopeObj().reset();
    m_driveSubsystem.getGyroscopeObj().zeroYaw();
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
    SmartDashboard.putNumber("CurrentPosX", m_driveSubsystem.getPose2d().getX());
    SmartDashboard.putNumber("CurrentPosY", m_driveSubsystem.getPose2d().getY());
    SmartDashboard.putNumber("CurrentPosRot", m_driveSubsystem.getPose2d().getRotation().getDegrees());
    SmartDashboard.putNumber("Controller POV", filteredController.getPOVButton());
    SmartDashboard.putBoolean("Controller POV Bool", filteredController.getPOVPressed());
    Logger.updateEntries();
  }
}
