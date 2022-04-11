// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Lookups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Auto.RunTrajectorySequence;
import frc.robot.commands.Groups.IntakeSequence;
import frc.robot.commands.Groups.ShootSequence;
import frc.robot.commands.Groups.ShooterAlignSequence;
import frc.robot.commands.Intake.RetractIntake;
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
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL1_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()));
                break;
            case "2BL1F1":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL1_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL1_F1")));
                break;
            case "2BL1F2":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL1_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL1_F2")));
                break;

            case "2BL2":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL2_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()));
                break;
            case "2BL2F1":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL2_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL1_F1")));
                break;
            case "2BL2F2":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL2_F")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL1_F2")));
                break;

            case "2BL3":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL3_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()));
                break;
            case "2BL3F1":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL3_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL3_F1")));
                break;
            case "2BL3F2":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL3_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL3_F2")));
                break;

            case "2BL4":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL4_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()));
                break;
            case "2BL4F1":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL4_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL4_F1")));
                break;
            case "2BL4F2":
                ret = new AutoRoutine(
                        new ParallelRaceGroup(
                                new RunTrajectorySequence(RobotContainer.getDriveSubsystem(),
                                        PathLookup.getContainer("2BL4_1")),
                                new IntakeSequence(RobotContainer.getIntakeSubsystem(),
                                        RobotContainer.getLifterSubsystem(), RobotContainer.getIntakeArmSubsystem())),
                        new ParallelRaceGroup(
                                new RetractIntake(RobotContainer.getIntakeArmSubsystem()),
                                new ShooterAlignSequence(RobotContainer.getDriveSubsystem(),
                                        RobotContainer.getHoodSubsystem()).withTimeout(2)),
                        new ShootSequence(RobotContainer.getLifterSubsystem()), new RunTrajectorySequence(
                                RobotContainer.getDriveSubsystem(), PathLookup.getContainer("2BL4_F2")));
                break;

            case "3BL4":
                ret = new AutoRoutine(

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
        }
        return ret;
    }
}
