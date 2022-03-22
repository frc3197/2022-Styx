// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups.Auto.NewAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.DriveSubsystem;
import java.util.List;

/** Add your docs here. */
public abstract class DriveSegmentBaseCommand extends SwerveControllerCommand{
    private DriveSubsystem _drivetrain;
    private Pose2d _initialPose;
    private Rotation2d _endRotation;
    private boolean _resetPostion;
    private Rotation2d _startRotation;

    /** Creates a new Auto10Feet. */
    public DriveSegmentBaseCommand(DriveSubsystem drivetrain,
                    List<Translation2d> waypoints,
                    Rotation2d startRotation,
                    Rotation2d endRotation,
                    boolean stopAtEnd,
                    boolean resetPosition) {
        super(getTrajectory(waypoints, getDefaultTrajectoryConfig(drivetrain, stopAtEnd)),
                drivetrain::getPose2d, // Functional interface to feed supplier
                drivetrain.getKinematics(),
                // Position controllers
                //TODO: ADJUST PID VALUES AS NECESSARY
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                getThetaController(),
                () -> endRotation,
                drivetrain::updateStates,
                drivetrain);
        _drivetrain = drivetrain;
        var firstWaypoint = waypoints.get(0);
        _initialPose = new Pose2d(firstWaypoint.getX(), firstWaypoint.getY(), startRotation); //getTrajectoryRotation(waypoints));
        _endRotation = endRotation;
        _resetPostion = resetPosition;
        _startRotation = startRotation;
    }

    @Override
    public void initialize() {
        super.initialize();
        if (!_resetPostion)
            _initialPose = new Pose2d(_drivetrain.getPose2d().getX(), _drivetrain.getPose2d().getY(), _startRotation);
        // Reset odometry to the starting pose of the trajectory.
        _drivetrain.resetOdometry(_initialPose);

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _drivetrain.resetOdometry(new Pose2d(_drivetrain.getPose2d().getX(),_drivetrain.getPose2d().getY(),_endRotation));
    }

    private static ProfiledPIDController getThetaController() {
        var profileConstraints = new TrapezoidProfile.Constraints(
                Constants.subsystems.H_Auto.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                Constants.subsystems.H_Auto.MAX_ANGULAR_ACCELERATION * Math.PI / 180 * 5);
        //TODO: ADJUST, SHOULD BE SIMILAR AS 1732 USES L2
        var thetaController = new ProfiledPIDController(1, 0, 0.01, profileConstraints);
        thetaController.enableContinuousInput(Math.PI * -1, Math.PI);
        return thetaController;
    }

    private static TrajectoryConfig getDefaultTrajectoryConfig(DriveSubsystem drivetrain, boolean stopAtEnd) {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.subsystems.H_Auto.MAX_VELOCITY_METERS_PER_SECOND,
                Constants.subsystems.H_Auto.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED*3);
        // Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(drivetrain.getKinematics());
        if (stopAtEnd)
            config.setEndVelocity(0.0);
        return config;
    }

    private static Trajectory getTrajectory(List<Translation2d> waypoints, TrajectoryConfig config) {
        if (waypoints.size() < 2) {
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    List.of(),
                    new Pose2d(0, 0, new Rotation2d(0)),
                    config);
        }
        var rotation = getTrajectoryRotation(waypoints);
        var interiorPoints = waypoints.subList(1, waypoints.size() - 1);
        var startPoint = waypoints.get(0);
        var endPoint = waypoints.get(waypoints.size() - 1);
        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(startPoint.getX(), startPoint.getY(), rotation),
                interiorPoints,
                new Pose2d(endPoint.getX(), endPoint.getY(), rotation),
                config);
    }

    private static Rotation2d getTrajectoryRotation(List<Translation2d> waypoints) {
        if (waypoints.size() < 2)
            return Rotation2d.fromDegrees(0);
        var startPoint = waypoints.get(0);
        var endPoint = waypoints.get(waypoints.size() - 1);
        double xdist = endPoint.getX() - startPoint.getX();
        double ydist = endPoint.getY() - startPoint.getY();
        double angle = Math.atan2(ydist, xdist);
        return new Rotation2d(angle);
    }
    //TODO: ADJUST WAYPOINTS AS NEEDED
    protected static final double scaler = 1;
    protected static final Pose2d WAYPOINT_A = new Pose2d(2.9238 * scaler, 0.41186 * scaler, Rotation2d.fromDegrees(-21));
    protected static final Pose2d WAYPOINT_B = new Pose2d(0.2910 * scaler, 0.0 * scaler, Rotation2d.fromDegrees(180));
    protected static final Pose2d WAYPOINT_C = new Pose2d(1.653747 - 0.2 * scaler, 2.67440* scaler, Rotation2d.fromDegrees(180-112));
    protected static final Pose2d WAYPOINT_D = new Pose2d(2.3598 - 0.6 * scaler, 1.5827 + 0.6* scaler, Rotation2d.fromDegrees(-42));
    protected static final Pose2d WAYPOINT_E = new Pose2d(1.51203 - 0.2 * scaler, 6.7617 + 0.5 * scaler, Rotation2d.fromDegrees(-45)); //0.8
    protected static final Pose2d WAYPOINT_F = new Pose2d(5.16403 * scaler, 2.18839 * scaler, Rotation2d.fromDegrees(-135));
    protected static final Pose2d WAYPOINT_G = new Pose2d(5.82539 +  0.3 * scaler, 2.8711 + 0.3 * scaler, Rotation2d.fromDegrees(-135));
    protected static final Pose2d WAYPOINT_H = new Pose2d(1.7778 * scaler, 0 * scaler, Rotation2d.fromDegrees(180));
    protected static final Pose2d WAYPOINT_I = new Pose2d(4.5541 * scaler, 1.21188 * scaler, Rotation2d.fromDegrees(-111));
    protected static final Pose2d WAYPOINT_J = new Pose2d(1.8958 * scaler, 0.8403 * scaler, Rotation2d.fromDegrees(-21));
    protected static final Pose2d WAYPOINT_K = new Pose2d(0.651352 * scaler, 1.2713 * scaler, Rotation2d.fromDegrees(-21));
    protected static final Pose2d WAYPOINT_L = new Pose2d(0.634897 * scaler, 0, Rotation2d.fromDegrees(0));

    protected static final Pose2d WAYPOINT_X = new Pose2d(0 * scaler, 0 * scaler, Rotation2d.fromDegrees(0));
    protected static final Pose2d WAYPOINT_Z = new Pose2d(2 * scaler, 0 * scaler, Rotation2d.fromDegrees(180));


    protected static final Pose2d WAYPOINT_A1 = new Pose2d(1.7778 * scaler, 0 * scaler, Rotation2d.fromDegrees(180));
    protected static final Pose2d WAYPOINT_B1 = new Pose2d(0.6410 * scaler, 0.0 * scaler, Rotation2d.fromDegrees(180));
    //protected static final Pose2d WAYPOINT_B2 = new Pose2d(0.2910 * scaler, 0.0 * scaler, Rotation2d.fromDegrees(180-112));
    //protected static final Pose2d WAYPOINT_C1 = new Pose2d(1.653747 - 0.2 * scaler, 2.67440* scaler, Rotation2d.fromDegrees(180-112));
    protected static final Pose2d WAYPOINT_B2 = new Pose2d(0,     0* scaler, Rotation2d.fromDegrees(0));
    protected static final Pose2d WAYPOINT_C1 = new Pose2d(2.916, 0* scaler, Rotation2d.fromDegrees(0));
    
    protected static final Pose2d WAYPOINT_C2 = new Pose2d(0,0* scaler, Rotation2d.fromDegrees(0));
    protected static final Pose2d WAYPOINT_D1 = new Pose2d(4.3,0 *scaler, Rotation2d.fromDegrees(0));




    
}