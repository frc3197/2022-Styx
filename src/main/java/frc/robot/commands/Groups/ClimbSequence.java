// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequence extends SequentialCommandGroup {
  /** Creates a new ClimbSequence. */
  ClimberSubsystem m_climberSubsystem;
  /** Creates a new ClimbSequence. */
  public ClimbSequence(ClimberSubsystem m_climberSubsystem) {
    this.m_climberSubsystem = m_climberSubsystem;
    addRequirements(m_climberSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands();
  }
}
