// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.Align;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.other.PIDConst;
import frc.robot.subsystems.DriveSubsystem;

public class ShooterXAlign extends CommandBase {
  DriveSubsystem driveSubsystem;
  PIDController xPID;
  PIDConst xPID_Constants;
  ChassisSpeeds curSpeeds,newSpeeds;
  double visionSetpoint, visionMeasurement, defaultTurnSpeed;

  /** Creates a new ShooterXAlign. */
  public ShooterXAlign(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    xPID_Constants = Constants.subsystems.swerve.xALIGN_PID;
    xPID = new PIDController(xPID_Constants.p, xPID_Constants.i, xPID_Constants.d);
    xPID.setTolerance(.5);
    defaultTurnSpeed = Constants.subsystems.shooter.defaultTurnSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curSpeeds = driveSubsystem.getChassisSpeeds();
    //TODO: FIX VISION ONCE LIMELIGHT IS UPDATED
    visionMeasurement = NetworkTableInstance.getDefault().getTable("limelight-rrone").getEntry("tx").getDouble(720);
    visionSetpoint = 0;
    if(visionMeasurement < 180){
      newSpeeds = new ChassisSpeeds(curSpeeds.vxMetersPerSecond,curSpeeds.vyMetersPerSecond,xPID.calculate(visionMeasurement, visionSetpoint));
    }
    else{
      newSpeeds = new ChassisSpeeds(curSpeeds.vxMetersPerSecond,curSpeeds.vyMetersPerSecond,defaultTurnSpeed);
    }
   
    driveSubsystem.drive(newSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
