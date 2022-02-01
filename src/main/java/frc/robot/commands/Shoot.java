// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LifterSubsystem;

public class Shoot extends CommandBase {
  LifterSubsystem lifter;
  Timer timer;
  boolean lowerBBState, upperBBState,isOver;
  /** Creates a new Shoot. */
  public Shoot(LifterSubsystem lifter) {
    this.lifter = lifter;
    timer = new Timer();
    isOver = false;
    addRequirements(lifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lifter.setBothMotors(0);
    lowerBBState = LifterSubsystem.getLowerBB();
    upperBBState = LifterSubsystem.getUpperBB();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = timer.get();
    // WAIT .25 SECONDS BEFORE FIRST SHOT
      if(curTime > .25){
      if(lowerBBState && upperBBState){
        lifter.releaseBoth();
        // THIS IS THE TIME IT TAKES TO ACTUALLY SHOOT
        Timer.delay(0);
        isOver = true;
      }
      else if(upperBBState){
        lifter.releaseUpper();
        // THIS IS THE TIME IT TAKES TO ACTUALLY SHOOT
        Timer.delay(0);
        isOver = true;
      }
      else{
        lifter.setLowerMotor(lifter.getLowerSpeed());
        lifter.setUpperMotor(lifter.getUpperSpeed());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lifter.setBothMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isOver||timer.get() > 2 ;
  }
}
