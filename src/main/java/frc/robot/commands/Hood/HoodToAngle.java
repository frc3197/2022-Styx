// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hood;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.other.extra_libraries.PIDConst;
import frc.robot.subsystems.Shooter.HoodSubsystem;

public class HoodToAngle extends CommandBase {
  HoodSubsystem hoodSubsystem;
  PIDController yPID;
  PIDConst yPID_Constants;
  ChassisSpeeds curSpeeds,newSpeeds;
  double angle;

  /** Creates a new ShooterYAlign. */
  public HoodToAngle(HoodSubsystem hoodSubsystem, double angle) {
    this.hoodSubsystem = hoodSubsystem;
    this.angle = angle;
    yPID_Constants = Constants.subsystems.swerve.yALIGN_PID;
    yPID = new PIDController(yPID_Constants.p, yPID_Constants.i, yPID_Constants.d);
    yPID.setTolerance(5);
    addRequirements(hoodSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hoodSubsystem.setHood(yPID.calculate(hoodSubsystem.getHoodPosition(), angle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.setHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yPID.atSetpoint();
  }
}
