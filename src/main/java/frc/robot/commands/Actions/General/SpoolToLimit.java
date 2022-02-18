// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.General;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class SpoolToLimit extends CommandBase {
  ClimberSubsystem climberSubsystem;
  boolean spoolLimitsBack, spoolLimitsForward;
  String direction;

  /** Creates a new ArmToLimit. */
  public SpoolToLimit(ClimberSubsystem climberSubsystem, String direction) {
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
    spoolLimitsBack = climberSubsystem.getSL_Limits();
    spoolLimitsForward = climberSubsystem.getSU_Limits();

    if (!spoolLimitsForward && direction.equals("Up")) {
      climberSubsystem.setSpoolSpeed(Constants.subsystems.climber.armSpeed);
    }
    if (!spoolLimitsBack && direction.equals("Down")) {
      climberSubsystem.setSpoolSpeed(-Constants.subsystems.climber.armSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.setSpoolSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (direction.equals("Up") && spoolLimitsForward) {
      return true;
    } else if (direction.equals("Down") && spoolLimitsBack) {
      return true;
    } else {
      return false;
    }
  }

}
