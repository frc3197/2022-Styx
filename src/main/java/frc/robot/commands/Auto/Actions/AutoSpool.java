// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Actions;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoSpool extends CommandBase {
  //TODO: Write Auto Spool
  /** Creates a new AutoSpool. */
  public AutoSpool() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //NOTE: THIS SHOULD NOT END (USED IN RACE GROUP)
    return false;
  }
}
