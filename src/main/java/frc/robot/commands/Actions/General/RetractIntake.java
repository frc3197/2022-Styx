// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.General;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
//TODO: Test
public class RetractIntake extends CommandBase {
  IntakeSubsystem m_intakeSubsystem;
  DigitalInput upperLimit;
  /** Creates a new DeployIntake. */
  public RetractIntake(IntakeSubsystem m_intakeSubsystem) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    upperLimit = new DigitalInput(Constants.subsystems.intake.armUpperLimitID);
    addRequirements(m_intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(upperLimit.get()){cancel();}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {m_intakeSubsystem.useArm(-Constants.subsystems.intake.armSpeed);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {m_intakeSubsystem.useArm(0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return upperLimit.get();
  }
}
