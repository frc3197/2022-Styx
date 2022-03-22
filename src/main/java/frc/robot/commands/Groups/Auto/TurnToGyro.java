// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.other.PIDConst;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class TurnToGyro extends CommandBase {
  DriveSubsystem driveSubsystem;
  double curAng,tarAng;
  PIDController pid;
  PIDConst pidConst;
  /** Creates a new TurnToGyro. */
  public TurnToGyro(DriveSubsystem driveSubsystem, double tarAng) {
    this.driveSubsystem = driveSubsystem;
    this.tarAng = tarAng;
    pidConst = Constants.subsystems.swerve.xALIGN_PID;
    pid = new PIDController(pidConst.p, pidConst.i, pidConst.d);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curAng = driveSubsystem.getGyroscopeRotation().getDegrees();
    driveSubsystem.drive(new ChassisSpeeds(0,0, pid.calculate(curAng, tarAng)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
