// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.Align;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.subsystems.hood;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.util.PIDConst;

public class ShooterYAlign extends CommandBase {
  HoodSubsystem hoodSubsystem;
  PIDController yPID;
  PIDConst yPID_Constants;
  ChassisSpeeds curSpeeds,newSpeeds;
  double visionSetpoint, visionMeasurement;

  /** Creates a new ShooterYAlign. */
  public ShooterYAlign(HoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    yPID_Constants = Constants.subsystems.swerve.yALIGN_PID;
    yPID = new PIDController(yPID_Constants.p, yPID_Constants.i, yPID_Constants.d);
    
    addRequirements(hoodSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: FIX VISION ONCE LIMELIGHT IS UPDATED
    visionMeasurement = 0;
    visionSetpoint = 0;
    hoodSubsystem.setHood(ControlMode.PercentOutput, yPID.calculate(visionMeasurement, visionSetpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.setHood(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yPID.atSetpoint();
  }
}
