// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.Actions.Align.IntakeAlign;
import frc.robot.commands.Actions.Movement.DriveStraight;
import frc.robot.subsystems.Drive.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HuntBall extends ParallelRaceGroup {
  DriveSubsystem driveSubsystem;
  double time;
  /** Creates a new HuntBall. */
  public HuntBall(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IntakeAlign(driveSubsystem), new DriveStraight(driveSubsystem));
  }

  public HuntBall(DriveSubsystem driveSubsystem,double time) {
    this.driveSubsystem = driveSubsystem;
    this.time = time;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IntakeAlign(driveSubsystem), new DriveStraight(driveSubsystem,1, time));
  }
  public HuntBall(DriveSubsystem driveSubsystem,double time,double speed) {
    this.driveSubsystem = driveSubsystem;
    this.time = time;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IntakeAlign(driveSubsystem), new DriveStraight(driveSubsystem,speed, time));
  }
  public HuntBall(DriveSubsystem driveSubsystem, boolean stopAfterCollection) {
    this.driveSubsystem = driveSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelRaceGroup(new IntakeAlign(driveSubsystem, stopAfterCollection), new DriveStraight(driveSubsystem)));
  }

  public HuntBall(DriveSubsystem driveSubsystem,double time,double speed, boolean stopAfterCollection) {
    this.driveSubsystem = driveSubsystem;
    this.time = time;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IntakeAlign(driveSubsystem), new DriveStraight(driveSubsystem,speed, time));
  }
}
