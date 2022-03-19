// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups.Auto.NewAuto;

import java.util.List;

import frc.robot.subsystems.Drive.DriveSubsystem;

public class TestSegment extends DriveSegmentBaseCommand {
  public TestSegment(DriveSubsystem drivetrain) {
    super(drivetrain, 
        List.of(WAYPOINT_X.getTranslation(), WAYPOINT_Z.getTranslation()),
        WAYPOINT_X.getRotation(),
        WAYPOINT_Z.getRotation(),
        false, // stopAtEnd
        false); // resetPosition
  }
}