// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Align.IntakeAlign;
import frc.robot.commands.Drivetrain.DriveStraight;
import frc.robot.commands.Drivetrain.RunBasicTrajectory;
import frc.robot.commands.Lifter.Shoot;
import frc.robot.commands.Shooter.Spool;
import frc.robot.other.extra_libraries.AutoRoutine;
import frc.robot.other.extra_libraries.SetNewOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@SuppressWarnings("unused")
public class Auto_2B_3 extends AutoRoutine {
  /** Creates a new simple2ball. */

  public Auto_2B_3() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new SetNewOdometry(super.getDriveSubsystem(),
                new Pose2d(299.54, 78.13, new Rotation2d(Units.degreesToRadians(91.5)))),
            new ParallelRaceGroup(new Spool(super.getShooterSubsystem()),
                new SequentialCommandGroup(
                    new ParallelRaceGroup(new IntakeSequence(super.getIntakeSubsystem(), super.getLifterSubsystem(),super.getIntakeArmSubsystem()),
                        new SequentialCommandGroup(new IntakeAlign(super.getDriveSubsystem()),
                        new DriveStraight(super.getDriveSubsystem(), 1.75),
                        new ShooterAlignSequence(super.getDriveSubsystem(), super.getHoodSubsystem()))),
                        new Shoot(super.getLifterSubsystem()))))
    );
  }
}
