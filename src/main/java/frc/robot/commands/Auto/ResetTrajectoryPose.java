// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetTrajectoryPose extends InstantCommand {
  DriveSubsystem driveSubsystem;
  String path;
  PathPlannerTrajectory trajectory;
  PoseAtTime poseAtTime;

  public enum PoseAtTime{
    START,END
  }



  public ResetTrajectoryPose(DriveSubsystem driveSubsystem, String path, PoseAtTime poseAtTime) {
    this.driveSubsystem = driveSubsystem;
    this.path = path;
    this.poseAtTime = poseAtTime;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectory = PathPlanner.loadPath(path, 3, 3);
    if(poseAtTime.equals(PoseAtTime.START)){
    DriveSubsystem.getGyroscopeObj().setYaw(-trajectory.getInitialState().holonomicRotation.getDegrees());
    // DriveSubsystem.getGyroscopeObj().setAngleAdjustment(trajectory.getInitialState().holonomicRotation.getDegrees());
    driveSubsystem.resetOdometry(new Pose2d(trajectory.getInitialState().poseMeters.getTranslation(),trajectory.getInitialState().poseMeters.getRotation()));
  }
  else{
    DriveSubsystem.getGyroscopeObj().setYaw(-trajectory.getEndState().holonomicRotation.getDegrees());
    // DriveSubsystem.getGyroscopeObj().setAngleAdjustment(trajectory.getEndState().holonomicRotation.getDegrees());
    driveSubsystem.resetOdometry(new Pose2d(trajectory.getEndState().poseMeters.getTranslation(),trajectory.getEndState().holonomicRotation));

  }
}
}
