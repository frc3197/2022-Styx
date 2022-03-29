// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups.Auto.WaypointCommands;

import java.util.List;

import frc.robot.commands.Groups.Auto.NewAuto.DriveSegmentBaseCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class DriveC2D1 extends DriveSegmentBaseCommand {
  public DriveC2D1(DriveSubsystem drivetrain) {
    super(drivetrain, 
        List.of(WAYPOINT_C2.getTranslation(), WAYPOINT_D1.getTranslation()),
        WAYPOINT_C2.getRotation(),
        WAYPOINT_D1.getRotation(),
        false,
        true);
  }
}
