// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/** Add your docs here. */
public class PathContainer {
    String pathString;
    PathPlannerTrajectory trajectory;
    double maxSpeed,maxAcceleration,timeout;

    public PathContainer(String pathString, double[] speeds, double timeout){
        this.pathString = pathString;
        this.timeout = timeout;
        trajectory = PathPlanner.loadPath(pathString, speeds[1], speeds[2]);
    }

    public String getPathString() {
        return pathString;
    }

    public void setPathString(String pathString) {
        this.pathString = pathString;
    }

    public PathPlannerTrajectory getTrajectory() {
        return trajectory;
    }

    public void setTrajectory(PathPlannerTrajectory trajectory) {
        this.trajectory = trajectory;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }

    public double getTimeout() {
        return timeout;
    }

    public void setTimeout(double timeout) {
        this.timeout = timeout;
    }
}
