// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Lookups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Auto.RunTrajectorySequence;
import frc.robot.commands.Groups.IntakeSequence;
import frc.robot.commands.Groups.ShootSequence;
import frc.robot.commands.Groups.ShooterAlignSequence;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Lifter.ShootUpper;
import frc.robot.commands.Lifter.SpitBoth;
import frc.robot.commands.Lifter.SpitBoth.CargoReleaseSpeed;
import frc.robot.commands.Lifter.SpitLower;
import frc.robot.commands.Shooter.Spool;
import frc.robot.other.extra_libraries.AutoRoutine;
import frc.robot.subsystems.Drive.LogOdometry;

/** Add your docs here. */
public class AutoLookup {
    public static AutoRoutine getAuto(String name) {
        AutoRoutine ret;
        switch (name) {
            default:
            case "2BL1":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL1_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1.5)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1))));
                break;
            case "2BL1F1":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL1_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1.5)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL1_F1")))));
                break;
            case "2BL1F2":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL1_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1.5)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL1_F2")))));
                break;

            case "2BL2":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL2_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1.5)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1))));
                break;
            case "2BL2F1":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL2_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL1_F1")))));
                break;
            case "2BL2F2":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL2_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL1_F2")))));
                break;

            case "2BL3":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL3_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1.5)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1))));
                break;
            case "2BL3F3":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL3_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL3_F3")))));
                break;
            case "2BL3F4":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL3_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL3_F4")))));
                break;

            case "2BL4":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL4_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1))));
                break;
            case "2BL4F3":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL4_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL4_F3")))));
                break;
            case "2BL4F4":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()),
                        new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL4_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(1)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL4_F4")))));
                break;

            case "3BL4":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()), new ShooterAlignSequence(RobotContainer.getDriveSubsystem(), RobotContainer.getHoodSubsystem()).withTimeout(1)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1),
                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                        PathLookup.getContainer("3BL4_1")),
                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem()),
                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                        PathLookup.getContainer("3BL4_2")),
                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem()), 
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()), new ShooterAlignSequence(RobotContainer.getDriveSubsystem(), RobotContainer.getHoodSubsystem()).withTimeout(1)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1)
                );
                break;
                case "3BL4F":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()), new ShooterAlignSequence(RobotContainer.getDriveSubsystem(), RobotContainer.getHoodSubsystem()).withTimeout(1)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1),
                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                        PathLookup.getContainer("3BL4_1")),
                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem()),
                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                        PathLookup.getContainer("3BL4_2")),
                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem()), 
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()), new ShooterAlignSequence(RobotContainer.getDriveSubsystem(), RobotContainer.getHoodSubsystem()).withTimeout(1)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1),
                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                        PathLookup.getContainer("3BL4_F"))
                );
                break;
            case "4BL3":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()), new LogOdometry(),
                                new SequentialCommandGroup(

                                        new ParallelRaceGroup(
                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                        PathLookup.getContainer("4BL3_1")),
                                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                        RobotContainer.getLifterSubsystem(),
                                                        RobotContainer.getIntakeArmSubsystem())),
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(1),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1),
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("4BL3_2")),
                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("4BL3_3"))),
                                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                        RobotContainer.getLifterSubsystem(),
                                                        RobotContainer.getIntakeArmSubsystem())),
                                        new ParallelCommandGroup(
                                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                        PathLookup.getContainer("4BL3_4"))),
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(1),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1.5)

                                )));
                break;
            case "4BL3F":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()), new LogOdometry(),
                                new SequentialCommandGroup(

                                        new ParallelRaceGroup(
                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                        PathLookup.getContainer("4BL3_1")),
                                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                        RobotContainer.getLifterSubsystem(),
                                                        RobotContainer.getIntakeArmSubsystem())),
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(1),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1),
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("4BL3_2")),
                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("4BL3_3"))),
                                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                        RobotContainer.getLifterSubsystem(),
                                                        RobotContainer.getIntakeArmSubsystem())),
                                        new ParallelCommandGroup(
                                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                        PathLookup.getContainer("4BL3_4"))),
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(1),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1.5),
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("4BL3_F"))

                                )));
                break;
                case "4BL4":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()), new LogOdometry(),
                                new SequentialCommandGroup(

                                        new ParallelRaceGroup(
                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                        PathLookup.getContainer("4BL4_1")),
                                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                        RobotContainer.getLifterSubsystem(),
                                                        RobotContainer.getIntakeArmSubsystem())),
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(.75),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1),
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("4BL4_2"))),
                                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                        RobotContainer.getLifterSubsystem(),
                                                        RobotContainer.getIntakeArmSubsystem())),
                                        new ParallelCommandGroup(
                                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                        PathLookup.getContainer("4BL4_3"))),
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(.75),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1.25)

                                )));
                break;
                case "4BL4F":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(new Spool(RobotContainer.getShooterSubsystem()), new LogOdometry(),
                                new SequentialCommandGroup(

                                        new ParallelRaceGroup(
                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                        PathLookup.getContainer("4BL4_1")),
                                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                        RobotContainer.getLifterSubsystem(),
                                                        RobotContainer.getIntakeArmSubsystem())),
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(1),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(.75),
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("4BL4_2"))),
                                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                        RobotContainer.getLifterSubsystem(),
                                                        RobotContainer.getIntakeArmSubsystem())),
                                        new ParallelCommandGroup(
                                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                        PathLookup.getContainer("4BL4_3"))),
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(.75),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1.25)

                                )));
                break;
            case "5BL4":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(.75),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(.5)),
                                new Spool(RobotContainer.getShooterSubsystem())),
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("5BL4_1")),
                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                                new ParallelDeadlineGroup(
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("5BL4_2")),
                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem()))),
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(.6),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1)),
                                new Spool(RobotContainer.getShooterSubsystem()),
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem())),
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("5BL4_3")),
                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem())),
                                //new WaitCommand(1),
                                new ParallelCommandGroup(
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("5BL4_5")),
                                        new SequentialCommandGroup(
                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(), RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem()).withTimeout(1),
                                        new RetractIntake(RobotContainer.getIntakeArmSubsystem())))),
                        new ParallelDeadlineGroup(
                            new SequentialCommandGroup(
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(.5),
                                new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1)),
                        new Spool(RobotContainer.getShooterSubsystem()))
                        );
                break;
                case "5BL4F":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(.75),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(.5)),
                                new Spool(RobotContainer.getShooterSubsystem())),
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("5BL4_1")),
                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                                new ParallelDeadlineGroup(
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("5BL4_2")),
                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem()))),
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(.6),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1)),
                                new Spool(RobotContainer.getShooterSubsystem()),
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem())),
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("5BL4_3")),
                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem())),
                                //new WaitCommand(1),
                                new ParallelCommandGroup(
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("5BL4_5")),
                                        new SequentialCommandGroup(
                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(), RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem()).withTimeout(1),
                                        new RetractIntake(RobotContainer.getIntakeArmSubsystem())))),
                        new ParallelDeadlineGroup(
                            new SequentialCommandGroup(
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(.5),
                                new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1)),
                        new Spool(RobotContainer.getShooterSubsystem())),
                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                            PathLookup.getContainer("5BL4_F")));
                break;
                case "2BFEN":
                ret = new AutoRoutine(
                new ParallelRaceGroup(
                new Spool(RobotContainer.getShooterSubsystem()), new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("2BFEN_1")),
                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem())),
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(), RobotContainer.getHoodSubsystem()).withTimeout(1),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1))),
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("2BFEN_2")),new RunTrajectorySequence(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BFEN_3")),new RunTrajectorySequence(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BFEN_4"))),
                                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem())),
                                                new InstantCommand(),new SpitBoth(RobotContainer.getIntakeSubsystem(),RobotContainer.getLifterSubsystem(),CargoReleaseSpeed.SLOW).withTimeout(1.5),new ParallelCommandGroup(new RunTrajectorySequence(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BFEN_F")), new RetractIntake(RobotContainer.getIntakeArmSubsystem()))
                );        
                break;
                case "2BFEN_ALT":
                ret = new AutoRoutine(
                new ParallelRaceGroup(
                new Spool(RobotContainer.getShooterSubsystem()), new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("2BFEN_1")),
                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem())),
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(), RobotContainer.getHoodSubsystem()).withTimeout(1),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1))),
                                        new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                PathLookup.getContainer("2BFEN_2")),new RunTrajectorySequence(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BFEN_3A")),new RunTrajectorySequence(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BFEN_4A")) ),
                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem())),
                                        new SpitBoth(RobotContainer.getIntakeSubsystem(),RobotContainer.getLifterSubsystem(),CargoReleaseSpeed.SLOW).withTimeout(1.5),new ParallelCommandGroup(new RunTrajectorySequence(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BFEN_FA")), new RetractIntake(RobotContainer.getIntakeArmSubsystem()))
                );                
                break;
                
                case "2B_HAN1":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                        new Spool(RobotContainer.getShooterSubsystem()), new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                        PathLookup.getContainer("2BHAN_1")),
                                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                        RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem())),
                                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(), RobotContainer.getHoodSubsystem()).withTimeout(1),
                                                new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1))),new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("2BHAN_2")),
                                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                        PathLookup.getContainer("2BHAN1_3"))),
                                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                                RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem())),
                                                        new SpitBoth(RobotContainer.getIntakeSubsystem(), RobotContainer.getLifterSubsystem(), CargoReleaseSpeed.SLOW),
                                                        new ParallelCommandGroup(new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("2BHAN1_F")))
                                                );
                break;
                case "2B_HAN2":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                        new Spool(RobotContainer.getShooterSubsystem()), new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                        PathLookup.getContainer("2BHAN_1")),
                                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                        RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem())),
                                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(), RobotContainer.getHoodSubsystem()).withTimeout(1),
                                                new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(1))),new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("2BHAN_2")),
                                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                        PathLookup.getContainer("2BHAN2_3")),
                                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                        PathLookup.getContainer("2BHAN2_4"))),
                                                        new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                                                RobotContainer.getLifterSubsystem(),RobotContainer.getIntakeArmSubsystem())),
                                                        new SpitBoth(RobotContainer.getIntakeSubsystem(), RobotContainer.getLifterSubsystem(), CargoReleaseSpeed.SLOW),
                                                        new ParallelCommandGroup(new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("2BHAN2_F")))
                                                );
                break;
                case "2E_STE.4":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(.75),
                                        new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(.5)),
                                new Spool(RobotContainer.getShooterSubsystem())),
                new ParallelRaceGroup(
                        new IntakeSequence(RobotContainer.getIntakeSubsystem(), RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem()),
                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(), 
                        PathLookup.getContainer("2E_STE.4.1"))
                        ),
                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(), 
                PathLookup.getContainer("2E_STE.4.2")
                ),
                new SpitLower(RobotContainer.getIntakeSubsystem(), RobotContainer.getLifterSubsystem(), SpitLower.CargoReleaseSpeed.FAST).withTimeout(2.5),
                new ParallelRaceGroup(
                        new IntakeSequence(RobotContainer.getIntakeSubsystem(), RobotContainer.getLifterSubsystem()),
                        new RunTrajectorySequence(RobotContainer.getDriveSubsystem(), 
                        PathLookup.getContainer("2E_STE.4.3"))
                ),
                new ParallelDeadlineGroup(new SequentialCommandGroup(
                new ParallelDeadlineGroup(new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                        PathLookup.getContainer("2E_STE.4.4")), 
                        new RetractIntake(RobotContainer.getIntakeArmSubsystem())
                ),
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(.75),
                                new ShootSequence(RobotContainer.getLifterSubsystem()).withTimeout(.5))
                        ), new Spool(RobotContainer.getShooterSubsystem()))),
                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                        PathLookup.getContainer("2E_STE.4.F")
                )

                );
                break;

                case "1E.2.1":
                        ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new ParallelRaceGroup(
                                                new IntakeSequence(RobotContainer.getIntakeSubsystem(), RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem()),
                                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(), 
                                                PathLookup.getContainer("1E.2.1"))),
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                                RobotContainer.getHoodSubsystem()).withTimeout(.75),
                                        new ShootUpper(RobotContainer.getLifterSubsystem()).withTimeout(.5)),
                                new Spool(RobotContainer.getShooterSubsystem())
                                ),
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(), 
                                PathLookup.getContainer("1E.2.2")),
                                new SpitLower(RobotContainer.getIntakeSubsystem(), RobotContainer.getLifterSubsystem(), SpitLower.CargoReleaseSpeed.FAST).withTimeout(2.5).withTimeout(1)
                                
                        
                        );
                break;
                case "1.4FEN":
                        ret = new AutoRoutine(
                                new ParallelRaceGroup(
                                        new Spool(RobotContainer.getShooterSubsystem()),
                                        new SequentialCommandGroup(
                                        new ShooterAlignSequence(RobotContainer.getDriveSubsystem(), RobotContainer.getHoodSubsystem()).withTimeout(5),
                                        new ShootSequence(RobotContainer.getLifterSubsystem())
                                        )
                                )
                        );
                break;
        
        }
        return ret;
    }
}
