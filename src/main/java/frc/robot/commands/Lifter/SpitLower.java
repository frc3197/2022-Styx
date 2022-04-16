// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lifter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.LifterSubsystem;

public class SpitLower extends CommandBase {
  IntakeSubsystem intakeSubsystem;
  LifterSubsystem lifterSubsystem;
  CargoReleaseSpeed cargoReleaseSpeed;


  /** Creates a new SpitLower. */
  public SpitLower(IntakeSubsystem intakeSubsystem, LifterSubsystem lifterSubsystem, CargoReleaseSpeed cargoReleaseSpeed) {
    this.lifterSubsystem = lifterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.cargoReleaseSpeed = cargoReleaseSpeed;
    addRequirements(intakeSubsystem, lifterSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.useIntake(-getIntakeReleaseSpeed(cargoReleaseSpeed));
    lifterSubsystem.setfeederMotor(-getLifterReleaseSpeed(cargoReleaseSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.useIntake(0);
    lifterSubsystem.setfeederMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  private double getIntakeReleaseSpeed(CargoReleaseSpeed cargoReleaseSpeed){
    if(cargoReleaseSpeed.equals(CargoReleaseSpeed.FAST)){
      return 1;
    }
    else if(cargoReleaseSpeed.equals(CargoReleaseSpeed.NORMAL)){
      return .7;
    }
    else{
      return .4;
    }
  }
    private double getLifterReleaseSpeed(CargoReleaseSpeed cargoReleaseSpeed){
      if(cargoReleaseSpeed.equals(CargoReleaseSpeed.FAST)){
        return -.8;
      }
      else if(cargoReleaseSpeed.equals(CargoReleaseSpeed.NORMAL)){
        return -.6;
      }
      else{
        return -.4;
      }
    }
    public enum CargoReleaseSpeed{
      FAST,
      NORMAL,
      SLOW  
    }
}
