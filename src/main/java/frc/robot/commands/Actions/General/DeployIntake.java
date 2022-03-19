// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.General;

import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeArm;
public class DeployIntake extends CommandBase {
  IntakeArm m_intakeSubsystem;
  private SparkMaxLimitSwitch lowerLimit;

  /** Creates a new DeployIntake. */
  public DeployIntake(IntakeArm m_intakeSubsystem) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    lowerLimit = m_intakeSubsystem.getArmMotor().getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    addRequirements(m_intakeSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {m_intakeSubsystem.useArm(.05);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {m_intakeSubsystem.useArm(0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lowerLimit.isPressed();
  }
}
