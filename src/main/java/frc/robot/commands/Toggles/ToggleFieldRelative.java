// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Toggles;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class ToggleFieldRelative extends InstantCommand {

  public ToggleFieldRelative() {

  }

  @Override
  public void initialize() {
    DriveSubsystem.setFieldRelative(DriveSubsystem.getFieldRelative() ? false : true);
  }
}
