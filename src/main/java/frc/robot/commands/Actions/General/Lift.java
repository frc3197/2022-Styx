// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.General;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LifterSubsystem;

public class Lift extends CommandBase {
  LifterSubsystem lifterSubsystem;
  boolean lowerBBState, upperBBState;
  /** Creates a new Lift. */
  public Lift(LifterSubsystem lifterSubsystem) {
    this.lifterSubsystem = lifterSubsystem;
    addRequirements(lifterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lowerBBState = LifterSubsystem.getLowerBB();
    upperBBState = LifterSubsystem.getUpperBB();
    lifterSubsystem.feed();
    if(upperBBState && lowerBBState){
      lifterSubsystem.setBothMotors(0);
    }
    else if(upperBBState && !lowerBBState){
      lifterSubsystem.setUpperMotor(0);
      lifterSubsystem.setLowerMotor(Constants.subsystems.lifter.lifterIntakeSpeed);
    }
    else{lifterSubsystem.setBothMotors(Constants.subsystems.lifter.lifterIntakeSpeed);}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lifterSubsystem.setBothMotors(0);
    lifterSubsystem.disableFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
