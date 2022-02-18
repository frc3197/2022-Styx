// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.General.Shoot;
import frc.robot.commands.Continuous.Spool;
import frc.robot.other.AutoRoutine;
import frc.robot.other.Wait;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_1B extends AutoRoutine {
  /** Creates a new Auto_1B. */
  public Auto_1B() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelRaceGroup(new SequentialCommandGroup(new Wait(.25), new Shoot(super.getLifterSubsystem())), new Spool(super.getShooterSubsystem(), 0)));
  }
}