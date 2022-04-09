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
    double maxSpeed, maxAcceleration, timeout;
    boolean first;

    public PathContainer(String pathString, double[] speeds, double timeout, boolean first) {
        this.pathString = pathString;
        this.timeout = timeout;
        this.first = first;
        trajectory = PathPlanner.loadPath(pathString, speeds[1], speeds[2]);
    }

    public String getPathString() {
        return pathString;
    }

    public PathPlannerTrajectory getTrajectory() {
        return trajectory;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getTimeout() {
        return timeout;
    }

    public boolean getFirst() {
        return first;
    }
}
