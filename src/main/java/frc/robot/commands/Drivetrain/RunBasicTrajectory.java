// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.other.extra_libraries.PathPlanner;
import frc.robot.other.extra_libraries.PathPlannerTrajectory;
import frc.robot.other.extra_libraries.PathPlannerTrajectory.PathPlannerState;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class RunBasicTrajectory extends CommandBase {
  private Pose2d currentPosition;
  private DriveSubsystem m_drivetrain;
  private PathPlannerTrajectory target;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private ProfiledPIDController rot_pid;
  private PathPlannerState state;
  private HolonomicDriveController hController;
  
  private final Timer timer = new Timer();

  public RunBasicTrajectory(DriveSubsystem m_drivetrain, String path) {
    this.m_drivetrain = m_drivetrain;
    rot_pid = Constants.auto.follower.ROT_PID_CONTROLLER;
    target = PathPlanner.loadPath(path, 3, 3);
  }

  @Override
  public void initialize() {
    m_drivetrain.setPose2d(new Pose2d(target.getInitialState().poseMeters.getX(),target.getInitialState().poseMeters.getY(), target.getInitialState().holonomicRotation));
    rot_pid.enableContinuousInput(-Math.PI, Math.PI);
    hController = new HolonomicDriveController(Constants.auto.follower.X_PID_CONTROLLER,
        Constants.auto.follower.Y_PID_CONTROLLER, rot_pid);
    hController.setTolerance(new Pose2d(.1, .1, new Rotation2d(.1)));

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    var curTime = timer.get();
    state = (PathPlannerState) target.sample(curTime);
    currentPosition = m_drivetrain.getPose2d();
    speeds = hController.calculate(currentPosition, state, state.holonomicRotation);
    m_drivetrain.setAllStates(m_drivetrain.getKinematics().toSwerveModuleStates(speeds));
  }

  
  /** 
   * @return boolean
   */
  @Override
  public boolean isFinished() {
    return false;
    }

  
  /** 
   * @param interrupted
   */
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

}