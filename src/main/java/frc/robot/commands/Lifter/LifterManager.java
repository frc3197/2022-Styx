// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lifter;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter.LifterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LifterManager extends SelectCommand {
  LifterSubsystem m_lifterSubsystem;
  
  private enum LifterSelector {
    CARGO_LOWER,
    CARGO_UPPER,
    CARGO_BOTH,
    CARGO_NONE
  }
  
  public LifterManager(LifterSubsystem m_lifterSubsystem) {
    
    super( 
      Map.ofEntries(

      Map.entry(LifterSelector.CARGO_BOTH, new InstantCommand()),
      // DOES NOTHING IF HOLDING 2 CARGO
      Map.entry(LifterSelector.CARGO_UPPER,
          new SequentialCommandGroup(new UpperToLower(m_lifterSubsystem),
              new FeedToUpper(m_lifterSubsystem))),
      // RESTAGES IN UPPER TO PREVENT ACCIDENTAL SPITOUT
      Map.entry(LifterSelector.CARGO_LOWER, new FeedToUpper(m_lifterSubsystem)),
      // FEED TO UPPER
      Map.entry(LifterSelector.CARGO_NONE, new InstantCommand())),
      // DOES NOTHING IF NO CARGO HELD
  LifterManager::select);
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

private static LifterSelector select() {
  switch (LifterSubsystem.getLifterStateString()) {
    case "2 Cargo in Lifter":
      return LifterSelector.CARGO_BOTH;
    case "1 Cargo in Lifter - Upper":
      return LifterSelector.CARGO_UPPER;
    case "1 Cargo in Lifter - Lower":
      return LifterSelector.CARGO_LOWER;
    case "No Cargo in Lifter":
      return LifterSelector.CARGO_NONE;
    default:
      return LifterSelector.CARGO_NONE;
  }
}
}
