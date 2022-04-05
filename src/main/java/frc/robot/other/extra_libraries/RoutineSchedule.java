// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other.extra_libraries;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class RoutineSchedule extends CommandBase{
    Command commandToRun;

    public RoutineSchedule(Command commandToRun, Subsystem requiredSubsystem){
        this.commandToRun = commandToRun;
        addRequirements(requiredSubsystem);
    }

    @Override
  public void execute() {
    if(!commandToRun.isScheduled()){
        commandToRun.schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
