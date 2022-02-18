// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.Movement.RunBasicTrajectory;
import frc.robot.other.AutoRoutine;
import frc.robot.other.SetNewOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_3B_3 extends AutoRoutine {
  public Auto_3B_3() {
    addCommands(
      
        new SequentialCommandGroup(
          new SetNewOdometry(super.getDriveSubsystem(),
            new Pose2d(300.00, 80.71, new Rotation2d(Units.degreesToRadians(88.24)))),
            new ParallelRaceGroup(new IntakeSequence(super.getIntakeSubsystem(), super.getLifterSubsystem()),
              new RunBasicTrajectory(super.getDriveSubsystem(), "3 ball #1.1")),
            new ShooterAlignSequence(super.getDriveSubsystem(), super.getHoodSubsystem()),
            new ShootSequence(super.getShooterSubsystem(), super.getLifterSubsystem()),
            new ParallelRaceGroup(new IntakeSequence(super.getIntakeSubsystem(), super.getLifterSubsystem()),
              new RunBasicTrajectory(super.getDriveSubsystem(), "3 ball #1.2")),
            new ShootSequence(super.getShooterSubsystem(), super.getLifterSubsystem())
        )
    );
  }
}
