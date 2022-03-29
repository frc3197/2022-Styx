// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter.HoodSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetHoodDefault extends InstantCommand {
  HoodSubsystem hoodSubsystem;
  int defaultVal;
  public SetHoodDefault(HoodSubsystem hoodSubsystem, int defaultVal) {
    this.hoodSubsystem = hoodSubsystem;
    this.defaultVal = defaultVal;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hoodSubsystem.setEncoderVal(defaultVal);
  }
}
