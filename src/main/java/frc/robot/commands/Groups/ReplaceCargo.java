// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Align.IntakeAlign;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.commands.Lifter.SpitLower;
import frc.robot.commands.Lifter.SpitLower.CargoReleaseSpeed;
import frc.robot.other.extra_libraries.AutoRoutine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReplaceCargo extends AutoRoutine {
  /** Creates a new ReplaceCargo. */
  public ReplaceCargo(CargoReleaseSpeed cSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(new IntakeAlign(getDriveSubsystem()), new DeployIntake(getIntakeArmSubsystem())),
        new SpitLower(getIntakeSubsystem(), getLifterSubsystem(), cSpeed));
  }
}
