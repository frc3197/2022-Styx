// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.Align;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.other.PIDConst;
import frc.robot.subsystems.DriveSubsystem;

public class IntakeAlign extends CommandBase {
  DriveSubsystem driveSubsystem;
  PIDController xPID;
  PIDConst xPID_Constants;
  ChassisSpeeds curSpeeds, newSpeeds;
  double visionSetpoint, visionMeasurement;
  PhotonTrackedTarget visionTarget;
  PhotonCamera cam = new PhotonCamera(Constants.subsystems.intake.camName);
  /** Creates a new IntakeAlign. */
  public IntakeAlign(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    xPID_Constants = Constants.subsystems.swerve.xALIGN_PID;
    xPID = new PIDController(xPID_Constants.p, xPID_Constants.i, xPID_Constants.d);
  
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curSpeeds = driveSubsystem.getChassisSpeeds();
    visionTarget = cam.getLatestResult().getBestTarget();
    visionMeasurement = visionTarget.getPitch();
    visionSetpoint = 0;
    if(visionTarget != null){
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
    driveSubsystem.drive(new ChassisSpeeds(0,0,0));}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xPID.atSetpoint();
  }
}
