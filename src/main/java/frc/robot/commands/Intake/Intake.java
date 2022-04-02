// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class Intake extends CommandBase {
  IntakeSubsystem intakeSubsystem;
  String direction;
  double delay;
  boolean delayOn;
  Timer timer = new Timer();
  /** Creates a new Intake. */
  public Intake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    direction = "Forward";
    delayOn = false;
  }
  public Intake(IntakeSubsystem intakeSubsystem,String direction) {
    this.intakeSubsystem = intakeSubsystem;
    this.direction = direction;
    delayOn = false;
  }
  
  public Intake(IntakeSubsystem intakeSubsystem,double delay) {
    this.intakeSubsystem = intakeSubsystem;
    this.delay = delay;
    direction = "Forward";
    delayOn = true;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {timer.reset();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(delayOn == true){
      timer.start();
      if(timer.get() > delay){
        //TODO: TEST
        if(direction.equals("Forward")){intakeSubsystem.useIntake(Constants.subsystems.intake.intakeSpeed + (0.2 * (RobotContainer.getDriver1().getYLeft(0) + RobotContainer.getDriver1().getXLeft(0) / 2)));}
    
  
        else if(direction.equals("Backward")){
          intakeSubsystem.useIntake(-Constants.subsystems.intake.intakeSpeed);
        }
      }
    }
    else{
    if(direction.equals("Forward")){intakeSubsystem.useIntake(Constants.subsystems.intake.intakeSpeed);}
    
  
  else if(direction.equals("Backward")){
    intakeSubsystem.useIntake(-Constants.subsystems.intake.intakeSpeed);
  }
  }}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.useIntake(0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
