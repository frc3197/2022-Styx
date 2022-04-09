// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lifter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.LifterSubsystem;

public class IntakeRumble extends CommandBase {
  /** Creates a new IntakeRumble. */
  public IntakeRumble() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(LifterSubsystem.getLifterStateString()){
      case "2 Cargo in Lifter":
      RobotContainer.getDriver1().setRumble(1);
      break;
      case "1 Cargo in Lifter - Upper":
      RobotContainer.getDriver1().setRumble(.25);
      break;
      case "1 Cargo in Lifter - Lower":
      RobotContainer.getDriver1().setRumble(.25);
      break;
      default:
      case "No Cargo in Lifter":
      RobotContainer.getDriver1().setRumble(0);
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
