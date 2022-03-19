// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.General.Lifter.Shoot;
import frc.robot.commands.Actions.Movement.ResetGyro;
import frc.robot.commands.Continuous.Spool;
import frc.robot.commands.Groups.IntakeSequence;
import frc.robot.commands.Groups.ShooterAlignSequence;
import frc.robot.commands.Groups.Auto.WaypointCommands.*;
import frc.robot.other.AutoRoutine;
import frc.robot.other.Wait;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_3B extends AutoRoutine {
  /** Creates a new Auto_3B. */
  public Auto_3B() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelRaceGroup(new Spool(super.getShooterSubsystem()),
        new SequentialCommandGroup(new ParallelCommandGroup(
             new ShooterAlignSequence(super.getDriveSubsystem(),
             super.getHoodSubsystem()).withTimeout(2),
            new Shoot(super.getLifterSubsystem()).withTimeout(1.5)),
            new ParallelRaceGroup(
                new IntakeSequence(getIntakeSubsystem(), getLifterSubsystem(), getIntakeArmSubsystem()),
                new SequentialCommandGroup(new DriveA1B1(super.getDriveSubsystem()),
                    new AutoTurn(super.getDriveSubsystem(), -112).withTimeout(1), new ResetGyro(getDriveSubsystem()),
                    new DriveB2C1(super.getDriveSubsystem()), new AutoTurn(getDriveSubsystem(), 30).withTimeout(.5),
                    new ShooterAlignSequence(getDriveSubsystem(), getHoodSubsystem()),new Shoot(getLifterSubsystem(), 1)

                )))));
  }
}
