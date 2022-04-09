// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.commands.Intake.Intake;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Lifter.Lift;
import frc.robot.subsystems.Intake.IntakeArm;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.LifterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSequence extends ParallelCommandGroup {
  LifterSubsystem lifterSubsystem;
  IntakeSubsystem intakeSubsystem;
  IntakeArm intakeArmSubsystem;
  BooleanSupplier booleanSupplier;
  String intakeDirection;

  // MUST TOGGLE THIS COMMAND!!!!!!!!!!!!!!!!!
  /** Creates a new IntakeAndLift. */
  public IntakeSequence(IntakeSubsystem intakeSubsystem, LifterSubsystem lifterSubsystem,
      IntakeArm intakeArmSubsystem) {
    this.lifterSubsystem = lifterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.intakeArmSubsystem = intakeArmSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new DeployIntake(intakeArmSubsystem), new Lift(lifterSubsystem), new Intake(intakeSubsystem, .2)
        //,new ConditionalCommand(new RumbleForTime(RobotContainer.getDriver1(), 1), new InstantCommand(),LifterSubsystem::newCargo)
        
        );
    // , new RumbleOnTrigger(RobotContainer.getDriver1(), new
    // Trigger(LifterSubsystem::getfeederBB))

  }

  public IntakeSequence(IntakeSubsystem intakeSubsystem, LifterSubsystem lifterSubsystem) {
    this.lifterSubsystem = lifterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new Lift(lifterSubsystem), new Intake(intakeSubsystem));

  }

  public IntakeSequence(IntakeSubsystem intakeSubsystem, LifterSubsystem lifterSubsystem, String direction) {
    this.lifterSubsystem = lifterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    intakeDirection = direction;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new Lift(lifterSubsystem), new Intake(intakeSubsystem, intakeDirection));

  }

  public IntakeSequence(IntakeSubsystem intakeSubsystem, LifterSubsystem lifterSubsystem, IntakeArm intakeArmSubsystem,
      BooleanSupplier booleanSupplier) {
    this.lifterSubsystem = lifterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.intakeArmSubsystem = intakeArmSubsystem;
    this.booleanSupplier = booleanSupplier;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new ParallelDeadlineGroup(new WaitForInput(booleanSupplier, "Closed"),
        new DeployIntake(intakeArmSubsystem), new Lift(lifterSubsystem), new Intake(intakeSubsystem)),
        new RetractIntake(intakeArmSubsystem));

  }
}
