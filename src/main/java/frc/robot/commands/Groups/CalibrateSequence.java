// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.subsystems.hood;
import frc.robot.commands.Actions.General.ArmToLimit;
import frc.robot.commands.Actions.General.CalibrateHood;
import frc.robot.commands.Actions.General.HoodToAngle;
import frc.robot.commands.Actions.General.RetractIntake;
import frc.robot.commands.Actions.General.SpoolToLimit;
import frc.robot.other.AutoRoutine;
import frc.robot.subsystems.Shooter.HoodSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CalibrateSequence extends AutoRoutine {
  /** Creates a new CalibrateSequence. */
  public CalibrateSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(new CalibrateHood(super.getHoodSubsystem()), new ArmToLimit(super.getClimberArmSubsystem(), "Backward"), new SpoolToLimit(super.getClimberSubsystem(), "Down"), new RetractIntake(super.getIntakeArmSubsystem())), new HoodToAngle(super.getHoodSubsystem(), 200));
  }
}
