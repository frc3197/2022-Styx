// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.ResetPath;
import frc.robot.commands.Drivetrain.ResetPath.TrajectoryTime;
import frc.robot.other.extra_libraries.AutoRoutine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathFollowSequence extends AutoRoutine {
  String path;
  /** Creates a new PathFollowSequence. */
  public PathFollowSequence(String path) {
    this.path = path;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ResetPath(getDriveSubsystem(), getTrajectory(), TrajectoryTime.START),
    new PPSwerveControllerCommand(
      getTrajectory(),
      getDriveSubsystem()::getPose2d, 
      getDriveSubsystem().getKinematics(),
      new PIDController(0, 0, 0),
      new PIDController(0, 0, 0), 
      getThetaController(),
      getDriveSubsystem()::setAllStates,
      getDriveSubsystem()),
    new ResetPath(getDriveSubsystem(),getTrajectory(),TrajectoryTime.END));
  }

  private ProfiledPIDController getThetaController(){
    ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(3, 3));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return thetaController;
  }
  private PathPlannerTrajectory getTrajectory(){
    return PathPlanner.loadPath(path, 3, 3);
  }
}
