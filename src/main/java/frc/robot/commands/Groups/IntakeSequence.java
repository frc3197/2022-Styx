// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Actions.General.DeployIntake;
import frc.robot.commands.Actions.General.Intake;
import frc.robot.commands.Actions.General.Lift;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSequence extends ParallelCommandGroup {
  LifterSubsystem lifterSubsystem;
  IntakeSubsystem intakeSubsystem;
  //MUST TOGGLE THIS COMMAND!!!!!!!!!!!!!!!!!
  /** Creates a new IntakeAndLift. */
  public IntakeSequence(IntakeSubsystem intakeSubsystem, LifterSubsystem lifterSubsystem) {
    this.lifterSubsystem = lifterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DeployIntake(intakeSubsystem),new Lift(lifterSubsystem), new Intake(intakeSubsystem));

    andThen(new DeployIntake(intakeSubsystem));


  }}


