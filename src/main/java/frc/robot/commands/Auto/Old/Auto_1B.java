// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Old;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Lifter.Shoot;
import frc.robot.commands.Shooter.Spool;
import frc.robot.other.extra_libraries.AutoRoutine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_1B extends AutoRoutine {
  /** Creates a new Auto_1B. */
  public Auto_1B() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(new SequentialCommandGroup(new Shoot(super.getLifterSubsystem(),2)), new Spool(super.getShooterSubsystem())));
  }
}
