// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Actions.General.Intake;
import frc.robot.commands.Actions.General.Lifter.Lift;
import frc.robot.commands.Actions.General.Lifter.Shoot;
import frc.robot.commands.Actions.Movement.DriveStraight;
import frc.robot.commands.Actions.Movement.KeepGyro;
import frc.robot.commands.Actions.Movement.ResetGyro;
import frc.robot.commands.Continuous.Spool;
import frc.robot.commands.Groups.HuntBall;
import frc.robot.commands.Groups.IntakeSequence;
import frc.robot.commands.Groups.ShootSequence;
import frc.robot.commands.Groups.ShooterAlignSequence;
import frc.robot.other.AutoRoutine;
import frc.robot.other.SetHoodDefault;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_5B extends AutoRoutine {
    /** Creates a new Auto_3B. */
    public Auto_5B() {
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
                                        new SequentialCommandGroup(new HuntBall(getDriveSubsystem(),true).withTimeout(2),
                                                new TurnToGyro(super.getDriveSubsystem(), -112).withTimeout(1),
                                                new ParallelRaceGroup(
                                                                new DriveStraight(getDriveSubsystem(), 3, .25, false)
                                                                        .withTimeout(.5),
                                                                new KeepGyro(getDriveSubsystem())),
                                                new HuntBall(getDriveSubsystem(), 1, 3).withTimeout(1.25))),
                                new SequentialCommandGroup(
                                        new TurnToGyro(getDriveSubsystem(), -35).withTimeout(.25),
                                        new ShooterAlignSequence(getDriveSubsystem(), getHoodSubsystem())
                                                .withTimeout(1),
                                        new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                        new ShootSequence(getLifterSubsystem()).withTimeout(1.25),
                                                        new TurnToGyro(getDriveSubsystem(), -92).withTimeout(1),
                                                        new ParallelRaceGroup(
                                                                new DriveStraight(getDriveSubsystem(), 3, 1.25, false)
                                                                        .withTimeout(1),
                                                                new KeepGyro(getDriveSubsystem())),
                                                        new HuntBall(getDriveSubsystem(), .4, 3), new WaitCommand(.5),
                                                        new TurnToGyro(getDriveSubsystem(), -75).withTimeout(.5),
                                                        new ParallelRaceGroup(new SequentialCommandGroup(
                                                                new DriveStraight(getDriveSubsystem(), -4, 1.25)
                                                                        .withTimeout(1),
                                                                new ShooterAlignSequence(getDriveSubsystem(),
                                                                        getHoodSubsystem()).withTimeout(1.5)),
                                                                new Lift(getLifterSubsystem())),
                                                        new Shoot(getLifterSubsystem())),
                                                new Intake(getIntakeSubsystem()))

                                ))));
    }
}
