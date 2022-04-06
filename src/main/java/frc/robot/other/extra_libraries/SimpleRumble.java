// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other.extra_libraries;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SimpleRumble extends CommandBase {
  FilteredController xboxController;
  BooleanSupplier booleanSupplier;
  double time = 0;
  Timer timer = new Timer();

  /** Creates a new RumbleForTime. */
  public SimpleRumble(FilteredController xboxController, BooleanSupplier booleanSupplier) {
    this.xboxController = xboxController;
    this.booleanSupplier = booleanSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(booleanSupplier.getAsBoolean() && timer.get() > time){
      xboxController.setRumble(1);
      timer.start();
      time = 1;
    }
    else{
      timer.stop();
      
      timer.reset();
      time = 0;
      xboxController.setRumble(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xboxController.setRumble(0);
    xboxController.setRumble(0);
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
