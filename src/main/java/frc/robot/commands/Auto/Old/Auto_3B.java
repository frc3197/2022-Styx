// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Old;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drivetrain.DriveStraight;
import frc.robot.commands.Drivetrain.KeepGyro;
import frc.robot.commands.Drivetrain.ResetGyro;
import frc.robot.commands.Groups.HuntBall;
import frc.robot.commands.Groups.IntakeSequence;
import frc.robot.commands.Groups.ShootSequence;
import frc.robot.commands.Groups.ShooterAlignSequence;
import frc.robot.commands.Lifter.Shoot;
import frc.robot.commands.Shooter.Spool;
import frc.robot.other.SetHoodDefault;
import frc.robot.other.extra_libraries.AutoRoutine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_3B extends AutoRoutine {
  /** Creates a new Auto_3B. */
  public Auto_3B() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetHoodDefault(getHoodSubsystem(), -340), new ResetGyro(getDriveSubsystem()),
    new ParallelRaceGroup(new Spool(super.getShooterSubsystem()),
            new SequentialCommandGroup(
                    new ShooterAlignSequence(super.getDriveSubsystem(),
                            super.getHoodSubsystem()).withTimeout(1.25),
                    new Shoot(super.getLifterSubsystem()).withTimeout(.75),
                    new ParallelRaceGroup(
                            new IntakeSequence(getIntakeSubsystem(), getLifterSubsystem(),
                                    getIntakeArmSubsystem()),
                            new SequentialCommandGroup(new HuntBall(getDriveSubsystem(),true).withTimeout(3),
                                    new TurnToGyro(super.getDriveSubsystem(), -112).withTimeout(3),
                                    new ParallelRaceGroup(new DriveStraight(getDriveSubsystem(), 3, .25, false)
                                                                        .withTimeout(.5),
                                                                new KeepGyro(getDriveSubsystem())),
                                    new HuntBall(getDriveSubsystem(), 1, 3).withTimeout(1.5), new WaitCommand(.5))),
                    new SequentialCommandGroup(
                            new TurnToGyro(getDriveSubsystem(), -35).withTimeout(1),
                            new ShooterAlignSequence(getDriveSubsystem(), getHoodSubsystem())
                                    .withTimeout(2),
                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                            new ShootSequence(getLifterSubsystem())))))));
  }
}
