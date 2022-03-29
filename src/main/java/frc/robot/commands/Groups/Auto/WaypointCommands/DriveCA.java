// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Groups.Auto.WaypointCommands;

import java.util.List;

import frc.robot.commands.Groups.Auto.NewAuto.DriveSegmentBaseCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class DriveCA extends DriveSegmentBaseCommand {
  public DriveCA(DriveSubsystem drivetrain) {
    super(drivetrain, 
    List.of(WAYPOINT_C.getTranslation(), WAYPOINT_A.getTranslation()),
        WAYPOINT_C.getRotation(),
        WAYPOINT_A.getRotation(),
        true,
        false);
  }
}
