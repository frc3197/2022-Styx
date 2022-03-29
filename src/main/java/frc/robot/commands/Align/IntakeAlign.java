// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.other.extra_libraries.PIDConst;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class IntakeAlign extends CommandBase {
  DriveSubsystem driveSubsystem;
  PIDController xPID;
  PIDConst xPID_Constants;
  ChassisSpeeds curSpeeds, newSpeeds;
  Timer timer = new Timer();
  double visionSetpoint, visionMeasurement, delay;
  /** Creates a new IntakeAlign. */
  public IntakeAlign(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    delay = 0;
    xPID_Constants = Constants.subsystems.swerve.xALIGN_PID;
    xPID = new PIDController(xPID_Constants.p, xPID_Constants.i, xPID_Constants.d);
    xPID.setTolerance(0);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  public IntakeAlign(DriveSubsystem driveSubsystem,double delay) {
    this.driveSubsystem = driveSubsystem;
    this.delay = delay;
    xPID_Constants = Constants.subsystems.swerve.xALIGN_PID;
    xPID = new PIDController(xPID_Constants.p, xPID_Constants.i, xPID_Constants.d);
    xPID.setTolerance(0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO:TEST
    timer.reset();
    timer.start();
    DriveSubsystem.setFieldRelative(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curSpeeds = driveSubsystem.getChassisSpeeds();
    visionMeasurement = driveSubsystem.getCamYaw();
    visionSetpoint = 0;
    if(DriveSubsystem.getCam().getLatestResult().hasTargets()){
      newSpeeds = new ChassisSpeeds(curSpeeds.vxMetersPerSecond,curSpeeds.vyMetersPerSecond,xPID.calculate(visionMeasurement, visionSetpoint));
    }
    else{
      newSpeeds = new ChassisSpeeds(curSpeeds.vxMetersPerSecond,curSpeeds.vyMetersPerSecond, Constants.subsystems.shooter.defaultTurnSpeed);
    }
     driveSubsystem.drive(newSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.setDriverMode(true);
    DriveSubsystem.setFieldRelative(true);
    driveSubsystem.drive(new ChassisSpeeds(0,0,0));}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(delay == 0){
      if(DriveSubsystem.getCam().getLatestResult().hasTargets()){
        return visionMeasurement == 0;
      }
      else{
        return false;
      }
    }
    else{
      return timer.get() > delay;
    }
}}