// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shoot;
import frc.robot.commands.Spool;
import frc.robot.other.AutoRoutine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class simple2ball extends AutoRoutine {
  /** Creates a new simple2ball. */

  public simple2ball() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // TODO: Get all these commmands created, written, and tested.
     // new ParallelRaceGroup(new Spool(), new SequentialCommandGroup(new IntakeAlign(), new DriveForwardDistance(super.getDriveSubsystem(), 0), new ShooterAlign(super.getShooterSubsystem()), new Shoot(super.getShooterSubsystem())))
      );
  }
}
