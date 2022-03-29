// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups.Auto.WaypointCommands;

import java.util.List;

import frc.robot.commands.Groups.Auto.NewAuto.DriveSegmentBaseCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class DriveA1B1 extends DriveSegmentBaseCommand {
  public DriveA1B1(DriveSubsystem drivetrain) {
    super(drivetrain, 
        List.of(WAYPOINT_A1.getTranslation(), WAYPOINT_B1.getTranslation()),
        WAYPOINT_A1.getRotation(),
        WAYPOINT_B1.getRotation(),
        false,
        true);
  }
}
