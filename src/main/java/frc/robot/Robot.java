// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Hood.CalibrateHood;
import frc.robot.subsystems.Climber.ClimberArm;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Shooter.HoodSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Command m_calibrateCommand = new CalibrateHood(RobotContainer.getHoodSubsystem());
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    System.out.print(Constants.subsystems.swerve.MAX_ANG_VEL_RAD);
    m_robotContainer.recalibrateGyroscope();
    m_calibrateCommand.schedule();
    CameraServer.startAutomaticCapture();
    DriveSubsystem.setDriverMode(true);

    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("MAX VEL RAD", Constants.subsystems.swerve.MAX_ANG_VEL_RAD);
    
    SmartDashboard.putNumber("MAX VEL RADa", Constants.subsystems.swerve.MAX_VEL_METERS);
    
    SmartDashboard.putNumber("MAX Acceleration RAD", Constants.auto.follower.ROT_PROFILE.maxAcceleration);
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_robotContainer.publishPosition();
   
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    
  }

  
  /** 
   * @param autonomousInit(
   */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    DriveSubsystem.setAlliancePipeline();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    if(m_calibrateCommand != null){
      m_calibrateCommand.cancel();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.resetOdometry();
    HoodSubsystem.resetHoodEncoder();
    //TODO: Test Driver Mode
    DriveSubsystem.setDriverMode(true);
    ClimberArm.resetEncoderValue();
    System.out.println(Constants.subsystems.swerve.MAX_VEL_METERS);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if(m_calibrateCommand != null){
      m_calibrateCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    if(m_calibrateCommand != null){
    m_calibrateCommand.schedule();
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
