// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.ResetTrajectoryPose.PoseAtTime;
import frc.robot.subsystems.Drive.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTrajectorySequence extends SequentialCommandGroup {
  DriveSubsystem driveSubsystem;
  PathContainer pathContainer;
  /** Creates a new RunTrajectorySequence. */
  public RunTrajectorySequence(DriveSubsystem driveSubsystem, PathContainer pathContainer) {
    this.pathContainer = pathContainer;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetTrajectoryPose(driveSubsystem, pathContainer, PoseAtTime.START), new FollowTrajectory(driveSubsystem, pathContainer).withTimeout(pathContainer.getTimeout()), new ResetTrajectoryPose(driveSubsystem, pathContainer, PoseAtTime.END));
  }
}
