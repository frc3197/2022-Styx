// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.General;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.LifterSubsystem;

public class Shoot extends CommandBase {
  LifterSubsystem lifter;
  Timer timer;
  double delay;
  /** Creates a new Shoot. */
  public Shoot(LifterSubsystem lifter, double delay) {
    this.lifter = lifter;
    this.delay = delay;
    timer = new Timer();
    addRequirements(lifter);
  }

  /** Creates a new Shoot. */
  public Shoot(LifterSubsystem lifter) {
    this.lifter = lifter;
    this.delay = 0;
    timer = new Timer();
    addRequirements(lifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    lifter.setBothMotors(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() > delay){
    lifter.setBothMotors(Constants.subsystems.lifter.lifterSpeed);
  }}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lifter.setBothMotors(0);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false ;
  }
}
