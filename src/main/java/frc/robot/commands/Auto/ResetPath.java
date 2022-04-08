// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetPath extends InstantCommand {
  public enum TrajectoryTime{
    START,END
  }
  DriveSubsystem driveSubsystem;
  PathPlannerTrajectory trajectory;
  TrajectoryTime trajTime;
  public ResetPath(DriveSubsystem driveSubsystem, PathPlannerTrajectory trajectory, TrajectoryTime trajTime) {
    this.trajectory = trajectory;
    this.driveSubsystem = driveSubsystem;
    this.trajTime = trajTime;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(trajTime.equals(TrajectoryTime.START)){
      DriveSubsystem.getGyroscopeObj().setYaw(trajectory.getInitialState().holonomicRotation.getDegrees());
      driveSubsystem.setPose2d(trajectory.getInitialState().poseMeters);
    }
    else{
      DriveSubsystem.getGyroscopeObj().setYaw(trajectory.getEndState().holonomicRotation.getDegrees());
      driveSubsystem.setPose2d(trajectory.getEndState().poseMeters);
    }
  }
}
