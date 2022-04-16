// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Align.ShooterXAlign;
import frc.robot.commands.Align.ShooterYAlign;
import frc.robot.commands.Shooter.Spool;
import frc.robot.other.extra_libraries.CancelAfterTimer;
import frc.robot.other.extra_libraries.SimpleRumble;
import frc.robot.other.extra_libraries.SimpleRumble.SIMPLERUMBLE_OPTIONS;
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
  ShooterXAlign shooterXAlign;
  ShooterYAlign shooterYAlign;
  double time;

  public ShooterAlignSequence(DriveSubsystem driveSubsystem, HoodSubsystem hoodSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    
    shooterXAlign = new ShooterXAlign(driveSubsystem);
    shooterYAlign = new ShooterYAlign(hoodSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(shooterXAlign, shooterYAlign,
        new SimpleRumble(RobotContainer.getDriver2(),
            () -> shooterYAlign.getAtSetpoint() && shooterXAlign.getAtSetpoint(), SIMPLERUMBLE_OPTIONS.NORMAL).perpetually());
          }

  public ShooterAlignSequence(DriveSubsystem driveSubsystem, HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    shooterXAlign = new ShooterXAlign(driveSubsystem);
    shooterYAlign = new ShooterYAlign(hoodSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(shooterXAlign, shooterYAlign,
        new SimpleRumble(RobotContainer.getDriver2(),
            () -> shooterYAlign.getAtSetpoint() && shooterXAlign.getAtSetpoint(), SIMPLERUMBLE_OPTIONS.NORMAL).perpetually(),
        new Spool(shooterSubsystem));
  }

  public ShooterAlignSequence(DriveSubsystem driveSubsystem, HoodSubsystem hoodSubsystem, double time) {
    this.driveSubsystem = driveSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.time = time;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelRaceGroup(new CancelAfterTimer(time),
        new ParallelCommandGroup(new ShooterXAlign(driveSubsystem), new ShooterYAlign(hoodSubsystem))));
  }
}
