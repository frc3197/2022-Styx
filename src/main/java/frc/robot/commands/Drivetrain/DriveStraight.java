// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class DriveStraight extends CommandBase {
  DriveSubsystem driveSubsystem;
  double time;
  ChassisSpeeds curSpeeds;
  Timer timer;
  double speed;
  boolean stopAtEnd;

  /** Creates a new DriveStraight. */
  public DriveStraight(DriveSubsystem driveSubsystem, double speed, double time) {
    this.driveSubsystem = driveSubsystem;
    timer = new Timer();
    this.time = time;
    this.speed = speed;
    stopAtEnd = true;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public DriveStraight(DriveSubsystem driveSubsystem, double speed, double time, boolean stopAtEnd) {
    this.driveSubsystem = driveSubsystem;
    timer = new Timer();
    this.time = time;
    this.speed = speed;
    this.stopAtEnd = stopAtEnd;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public DriveStraight(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    timer = new Timer();
    speed = 1;
    time = 99999;
    stopAtEnd = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.setFieldRelative(true);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curSpeeds = driveSubsystem.getChassisSpeeds();
    if (timer.get() < time) {
      driveSubsystem.drive(new ChassisSpeeds(speed, curSpeeds.vyMetersPerSecond, curSpeeds.omegaRadiansPerSecond));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stopAtEnd) {
      driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
