// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Auto.Actions.ShooterXAlign;
import frc.robot.commands.Auto.Actions.ShooterYAlign;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAlign extends ParallelCommandGroup {
  ShooterSubsystem shooterSubsystem;
  public ShooterAlign(ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //TODO: Test Shooter Align Sequence
    addCommands(new ShooterXAlign(), new ShooterYAlign());
  }
}
