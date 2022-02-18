// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.General.ArmToLimit;
import frc.robot.commands.Actions.General.RotateArm;
import frc.robot.commands.Actions.General.SpoolToLimit;
import frc.robot.other.AutoRoutine;
import frc.robot.other.ClimbType;
import frc.robot.other.Wait;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LevelUp extends AutoRoutine {
  /** Creates a new ClimbSequence. */
  ClimberSubsystem m_climberSubsystem;

  /** Creates a new ClimbSequence. */
  public LevelUp(ClimberSubsystem m_climberSubsystem, ClimbType climbType) {
    this.m_climberSubsystem = m_climberSubsystem;
    addRequirements(m_climberSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(new Wait(climbType.waitTime), new ArmToLimit(super.getClimberSubsystem(), "Backward")),
            new SpoolToLimit(super.getClimberSubsystem(), "Up")),
        new SpoolToLimit(super.getClimberSubsystem(), "Down"),
        new RotateArm(super.getClimberSubsystem(), climbType.fTicks),
        new SpoolToLimit(super.getClimberSubsystem(), "Up"), new RotateArm(super.getClimberSubsystem(), climbType.hook),
        new ParallelCommandGroup(new RotateArm(super.getClimberSubsystem(), climbType.unhook),
            new SpoolToLimit(super.getClimberSubsystem(), "Down")));
  }

}
