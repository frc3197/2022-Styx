// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.Manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.subsystems.climber;
import frc.robot.subsystems.ClimberSubsystem;

public class ManualSpool extends CommandBase {
  ClimberSubsystem climberSubsystem;
  String direction;
  /** Creates a new ManualRotateArm. */
  public ManualSpool(ClimberSubsystem climberSubsystem, String direction) {
    this.climberSubsystem = climberSubsystem;
    this.direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(direction.equals("Up")){
      climberSubsystem.setSpoolSpeed(.5);
    }
    else if(direction.equals("Down")){
      climberSubsystem.setSpoolSpeed(-.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {climberSubsystem.setSpoolSpeed(0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
