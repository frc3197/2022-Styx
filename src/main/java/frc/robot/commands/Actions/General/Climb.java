// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.General;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class Climb extends CommandBase {
  ClimberSubsystem climberSubsystem;
  String direction;
  boolean isReversed;
  double speed;
  /** Creates a new Climb. */
  public Climb(ClimberSubsystem climberSubsystem, String direction) {
    this.climberSubsystem = climberSubsystem;
    this.direction = direction;
    speed = Constants.subsystems.climber.spoolSpeed;
    addRequirements(climberSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(direction.equals("Up")){
      isReversed = false;
    }
    else if(direction.equals("Down")){
      isReversed = true;
    }
    else{isReversed = false;}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {climberSubsystem.setSpoolSpeed(isReversed ? -speed : speed );}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {climberSubsystem.setSpoolSpeed(0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
