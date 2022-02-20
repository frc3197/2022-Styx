// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.General;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;

public class CalibrateHood extends CommandBase {
  HoodSubsystem hood;

  /** Creates a new CalibrateHood. */
  public CalibrateHood(HoodSubsystem hood) {
    this.hood = hood;
    addRequirements(hood);
    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setHood(-Constants.subsystems.hood.hoodSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setHoodValue(0);
    hood.setHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(hood.getHoodBackLimit()==true){
        return true;
    }
    else{return false;}
  }
}
