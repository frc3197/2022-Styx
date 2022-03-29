// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lifter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.LifterSubsystem;

public class Lift extends CommandBase {
  LifterSubsystem lifterSubsystem;
  boolean feederBBState, lifterBBState;
  Timer timer = new Timer();
  /** Creates a new Lift. */
  public Lift(LifterSubsystem lifterSubsystem) {
    this.lifterSubsystem = lifterSubsystem;
    timer.reset();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederBBState = LifterSubsystem.getfeederBB();
    lifterBBState = LifterSubsystem.getlifterBB();
    if(lifterBBState && feederBBState){
      lifterSubsystem.setBothMotors(0);
    }
    else if(lifterBBState && !feederBBState){
      lifterSubsystem.setfeederMotor(-Constants.subsystems.lifter.feederSpeed);
      lifterSubsystem.setlifterMotor(0);
    }
    else{lifterSubsystem.setlifterMotor(Constants.subsystems.lifter.lifterSpeed);
    lifterSubsystem.setfeederMotor(-Constants.subsystems.lifter.feederSpeed);
    } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lifterSubsystem.setBothMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
