// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.General;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbToLimit extends CommandBase {
  /** Creates a new ClimbToLimit. */
  ClimberSubsystem climbersubsystem;
  int armDirection;
  String direction;
  boolean isFinished;
  public ClimbToLimit(ClimberSubsystem climbersubsystem, String direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.direction = direction;
    this.climbersubsystem = climbersubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(direction.equals("forward"))
    {
      armDirection = 1;
    }
    else if(direction.equals("back"))
    {
      armDirection = -1;
    }
    else
    {
      armDirection = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbersubsystem.setArmVoltage(Constants.subsystems.climber.armVoltage * armDirection);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbersubsystem.setArmVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(direction.equals("forward"))
    {
      if(climbersubsystem.getfLimit())
      {
        isFinished = true;
      }
    }
    else if(direction.equals("back"))
    {
      if(climbersubsystem.getbLimit())
      {
        isFinished = true;
      }
    }
    return isFinished;
  }
}
