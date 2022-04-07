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
  SIMPLERUMBLE_OPTIONS sOptions;
  double time = 0;
  double hold = 0;
  boolean booleanGet;
  Timer timer = new Timer();
  Timer holdInput = new Timer();

  /** Creates a new RumbleForTime. */
  public SimpleRumble(FilteredController xboxController, BooleanSupplier booleanSupplier,
      SIMPLERUMBLE_OPTIONS sOptions) {
    this.xboxController = xboxController;
    this.booleanSupplier = booleanSupplier;
    this.sOptions = sOptions;
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
    booleanGet = booleanSupplier.getAsBoolean();
    if (sOptions.equals(SIMPLERUMBLE_OPTIONS.NORMAL)) {

      if (booleanGet && timer.get() >= time) {
        xboxController.setRumble(1);
        time = 1;

        timer.start();
      } else {
        timer.reset();
        timer.stop();
        timer.reset();
        time = 0;
        xboxController.setRumble(0);
      }
    }
    else{
     if(booleanGet || holdInput.get() <= hold){
        hold = 2;
        holdInput.start();
        booleanGet = true;
     }
     else{
       holdInput.reset();
       booleanGet = false;
       hold = 0;
       holdInput.stop();
       holdInput.reset();
     }
      if (booleanGet && timer.get() >= time) {
        xboxController.setRumble(1);
        time = 1;

        timer.start();
      } else {
        timer.reset();
        timer.stop();
        timer.reset();
        time = 0;
        xboxController.setRumble(0);
      }
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

  public enum SIMPLERUMBLE_OPTIONS {
    NORMAL,
    HOLDBOOL
  }
}
