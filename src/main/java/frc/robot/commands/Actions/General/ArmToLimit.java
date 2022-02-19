// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.General;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberArm;
import frc.robot.subsystems.ClimberSubsystem;

public class ArmToLimit extends CommandBase {
  ClimberArm climberSubsystem;
  boolean armLimitsBack, armLimitsForward;
  String direction;

  /** Creates a new ArmToLimit. */
  public ArmToLimit(ClimberArm climberSubsystem, String direction) {
    this.climberSubsystem = climberSubsystem;
    this.direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armLimitsBack = climberSubsystem.getBackLimits();
    armLimitsForward = climberSubsystem.getFrontLimits();

    if (!armLimitsForward && direction.equals("Forward")) {
      climberSubsystem.setArmSpeed(Constants.subsystems.climber.armSpeed);
    }
    if (!armLimitsBack && direction.equals("Backward")) {
      climberSubsystem.setArmSpeed(-Constants.subsystems.climber.armSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.setArmVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (direction.equals("Forward") && armLimitsForward) {
      return true;
    } else if (direction.equals("Backward") && armLimitsBack) {
      return true;
    } else {
      return false;
    }
  }

}
