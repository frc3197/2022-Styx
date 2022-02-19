// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.Actions.Align.IntakeAlign;
import frc.robot.commands.Actions.General.Climb;
import frc.robot.commands.Actions.General.DeployIntake;
import frc.robot.commands.Actions.General.Intake;
import frc.robot.commands.Actions.General.Lift;
import frc.robot.commands.Actions.General.RetractIntake;
import frc.robot.commands.Actions.General.Shoot;
import frc.robot.commands.Actions.Manual.ManualRotateArm;
import frc.robot.commands.Actions.Manual.ManualSpool;
import frc.robot.commands.Continuous.DriveCommand;
import frc.robot.commands.Continuous.Spool;
import frc.robot.commands.Groups.Auto_1B;
import frc.robot.commands.Groups.Auto_2B_1;
import frc.robot.commands.Groups.Auto_2B_2;
import frc.robot.commands.Groups.Auto_2B_3;
import frc.robot.commands.Groups.IntakeSequence;
import frc.robot.commands.Groups.LevelUp;
import frc.robot.commands.Groups.ShooterAlignSequence;
import frc.robot.commands.Toggles.Defend;
import frc.robot.commands.Toggles.ToggleManualHood;
import frc.robot.other.ClimbType;
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
  @SuppressWarnings("rawtypes")
  private static SendableChooser m_autoChooser;
  @SuppressWarnings("rawtypes")
  private static SendableChooser m_allianceChooser; 

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
    m_autoChooser = new SendableChooser<>();
    //m_autoChooser.setDefaultOption("Nothing", null);
    //m_autoChooser.addOption("2Ball1", new Auto_2B_1());
    //m_autoChooser.addOption("2Ball2", new Auto_2B_2());
    //m_autoChooser.addOption("2Ball3", new Auto_2B_3());
   // m_autoChooser.addOption("1Ball", new Auto_1B());

    
    m_allianceChooser = new SendableChooser<>();
    m_allianceChooser.setDefaultOption("Nothing", null);
    m_allianceChooser.addOption("Red", "Red");
    m_allianceChooser.addOption("Blue", "Blue");
    
    SmartDashboard.putData(m_allianceChooser);
    SmartDashboard.putData(m_autoChooser);
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

    
    // DRIVER 1
    //new Button(m_controller1::getAButton).toggleWhenPressed(new Defend(m_driveSubsystem));
    new Button(m_controller1::getRightBumper).whenHeld(new IntakeAlign(m_driveSubsystem));
    //new Button(m_controller1::getStartButton).whenPressed(new ForceReleaseLower(m_lifterSubsystem, m_intakeSubsystem));
    //new Button(m_controller1::getBackButtonPressed).whenPressed(new ForceReleaseUpper(m_lifterSubsystem, m_shooterSubsystem,m_hoodSubsystem));
    new Button(filteredController1::getRightTriggerActive).whileHeld(new IntakeSequence(m_intakeSubsystem,m_lifterSubsystem));
    
    new Button(m_controller1::getBButton).whenHeld(new ManualRotateArm(m_climberSubsystem, "Forward"));
    new Button(m_controller1::getXButton).whenHeld(new ManualRotateArm(m_climberSubsystem, "Backward"));
    new Button(m_controller1::getAButton).whenHeld(new ManualSpool(m_climberSubsystem, "Up"));
    new Button(m_controller1::getAButton).whenHeld(new ManualSpool(m_climberSubsystem, "Down"));


    

    
    // DRIVER 2 
    new Button(filteredController2::getRightTriggerActive).whileHeld(new Spool(m_shooterSubsystem, Constants.subsystems.shooter.targetRPM));
    new Button(m_controller2::getRightBumper).whenHeld(new Shoot(m_lifterSubsystem));
    //new Button(m_controller2::getStartButtonPressed).whenPressed(new LevelUp(m_climberSubsystem, ClimbType.high));
    //new Button(m_controller2::getYButton).whenPressed(new Climb(m_climberSubsystem, "Up"));
    //new Button(m_controller2::getAButton).whenPressed(new Climb(m_climberSubsystem, "Down"));
    //new Button(m_controller2::getStartButton).whenPressed(new ForceReleaseLower(m_lifterSubsystem, m_intakeSubsystem));
    //new Button(m_controller2::getBackButtonPressed).whenPressed(new ForceReleaseUpper(m_lifterSubsystem, m_shooterSubsystem,m_hoodSubsystem));
    new Button(m_controller2::getLeftBumper).whenHeld(new ShooterAlignSequence(m_driveSubsystem, m_hoodSubsystem));
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
  public static SendableChooser getAllianceChooser(){
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

  //public static ClimberSubsystem getClimberSubsystem() {
    //return m_climberSubsystem;
  //}

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
