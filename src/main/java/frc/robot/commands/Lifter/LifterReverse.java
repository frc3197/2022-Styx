// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lifter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.LifterSubsystem;

public class LifterReverse extends CommandBase {
  LifterSubsystem lifterSubsystem;
  /** Creates a new LifterReverse. */
  public LifterReverse(LifterSubsystem lifterSubsystem) {
    this.lifterSubsystem = lifterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lifterSubsystem.setlifterMotor(-Constants.subsystems.lifter.lifterSpeed);
    lifterSubsystem.setfeederMotor(Constants.subsystems.lifter.feederSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lifterSubsystem.setBothMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
