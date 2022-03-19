// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.Movement;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class DriveStraight extends CommandBase {
  DriveSubsystem driveSubsystem;
  double time;
  ChassisSpeeds curSpeeds;
  Timer timer;

  /** Creates a new DriveStraight. */
  public DriveStraight(DriveSubsystem driveSubsystem, double time) {
    this.driveSubsystem = driveSubsystem;
    timer = new Timer();
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
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
      driveSubsystem.drive(new ChassisSpeeds(2, curSpeeds.vyMetersPerSecond, curSpeeds.omegaRadiansPerSecond));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
