// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.subsystems.shooter;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class SetShooterVoltage extends CommandBase {
  ShooterSubsystem shooterSubsystem;
  double input;
  /** Creates a new SetShooterVoltage. */
  public SetShooterVoltage(ShooterSubsystem shooterSubsystem, double input) {
    this.shooterSubsystem = shooterSubsystem;
    this.input = input;
    addRequirements(shooterSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setVoltage(input);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
