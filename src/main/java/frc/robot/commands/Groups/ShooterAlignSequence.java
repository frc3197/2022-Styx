// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Actions.Align.ShooterXAlign;
import frc.robot.commands.Actions.Align.ShooterYAlign;
import frc.robot.commands.Continuous.Spool;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAlignSequence extends ParallelCommandGroup {
  DriveSubsystem driveSubsystem;
  HoodSubsystem hoodSubsystem;
  ShooterSubsystem shooterSubsystem;
  public ShooterAlignSequence(DriveSubsystem driveSubsystem,HoodSubsystem hoodSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //TODO: Test Shooter Align Sequence
    addCommands(new ShooterXAlign(driveSubsystem), new ShooterYAlign(hoodSubsystem));
  }

  public ShooterAlignSequence(DriveSubsystem driveSubsystem,HoodSubsystem hoodSubsystem, ShooterSubsystem shooterSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //TODO: Test Shooter Align Sequence
    addCommands(new ShooterXAlign(driveSubsystem), new ShooterYAlign(hoodSubsystem), new Spool(shooterSubsystem));
  }
}
