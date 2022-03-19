// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.Align.IntakeAlign;
import frc.robot.commands.Actions.General.Lifter.Shoot;
import frc.robot.commands.Actions.Movement.DriveStraight;
import frc.robot.commands.Actions.Movement.RunBasicTrajectory;
import frc.robot.commands.Continuous.Spool;
import frc.robot.commands.Groups.IntakeSequence;
import frc.robot.commands.Groups.ShooterAlignSequence;
import frc.robot.other.AutoRoutine;
import frc.robot.other.SetNewOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@SuppressWarnings("unused")
public class Auto_2B extends AutoRoutine {
  /** Creates a new simple2ball. */

  public Auto_2B() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new SetNewOdometry(super.getDriveSubsystem(),
                new Pose2d(264.18, 234.06, new Rotation2d(Units.degreesToRadians(43.5)))),
            new ParallelRaceGroup(
                new Spool(super.getShooterSubsystem()),
                new SequentialCommandGroup(
                    new ParallelRaceGroup(new IntakeSequence(super.getIntakeSubsystem(), super.getLifterSubsystem(),super.getIntakeArmSubsystem()),
                        new SequentialCommandGroup(new IntakeAlign(super.getDriveSubsystem(),3),
                            new DriveStraight(super.getDriveSubsystem(), 1.75),
                        new ShooterAlignSequence(super.getDriveSubsystem(), super.getHoodSubsystem(),3))),
                        new Shoot(super.getLifterSubsystem(),3)))));
  }
}