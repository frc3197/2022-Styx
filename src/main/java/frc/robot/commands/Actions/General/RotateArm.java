// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.General;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.other.PIDConst;
import frc.robot.subsystems.Climber.ClimberArm;

public class RotateArm extends CommandBase {
  ClimberArm climberSubsystem;
  double initalPos,currentPos,targetVal;
  PIDController pid; 
  PIDConst pidConst;
  /** Creates a new RotateArm. */
  public RotateArm(ClimberArm climberSubsystem,double targetVal) {
    this.climberSubsystem = climberSubsystem;
    this.targetVal = targetVal;
    addRequirements(climberSubsystem);
    pidConst = Constants.subsystems.climber.climberArmCost;
    pid = new PIDController(pidConst.p, pidConst.i, pidConst.d);
    pid.setTolerance(Constants.subsystems.climber.armRotationTolerance);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initalPos = climberSubsystem.getArmEncoderValueLeft();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPos = climberSubsystem.getArmEncoderValueLeft();
    climberSubsystem.setArmVoltage(pid.calculate(currentPos, targetVal) * Constants.subsystems.climber.armMaxVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {climberSubsystem.setArmVoltage(0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
