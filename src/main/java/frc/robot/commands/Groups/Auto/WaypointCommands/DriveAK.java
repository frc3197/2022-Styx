// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups.Auto.WaypointCommands;

import java.util.List;

import frc.robot.commands.Groups.Auto.NewAuto.DriveSegmentBaseCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class DriveAK extends DriveSegmentBaseCommand {
  public DriveAK(DriveSubsystem drivetrain) {
    super(drivetrain, 
    List.of(WAYPOINT_A.getTranslation(), WAYPOINT_K.getTranslation()),
        WAYPOINT_A.getRotation(),
        WAYPOINT_K.getRotation(),
        false,
        false);
  }
}
