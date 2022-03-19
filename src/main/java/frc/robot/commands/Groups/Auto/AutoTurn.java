// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.other.PIDConst;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class AutoTurn extends CommandBase {
  DriveSubsystem driveSubsystem;
  double target,initialAngle,endAngle,currentAngle;
  PIDConst pidConst = Constants.subsystems.swerve.xALIGN_PID; 
  PIDController pid;
  /** Creates a new AutoTurn. */
  public AutoTurn(DriveSubsystem driveSubsystem,double target) {
    this.driveSubsystem = driveSubsystem;
    this.target = target;
    pid = new PIDController(pidConst.p-.02, pidConst.i, pidConst.d);
    pid.setTolerance(0);
    pid.enableContinuousInput(-180, 180);
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = driveSubsystem.getGyroscopeRotation().getDegrees();
    endAngle = initialAngle + target;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("InitialAngle", initialAngle);
    SmartDashboard.putNumber("CurrentAngle", currentAngle);
    SmartDashboard.putNumber("EndAngle", endAngle);
    currentAngle = driveSubsystem.getGyroscopeRotation().getDegrees();
    driveSubsystem.drive(new ChassisSpeeds(0, 0, pid.calculate(currentAngle, endAngle)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
