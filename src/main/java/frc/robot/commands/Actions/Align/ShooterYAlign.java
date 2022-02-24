// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.Align;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.other.PIDConst;
import frc.robot.other.RangeLookup;
import frc.robot.subsystems.Shooter.HoodSubsystem;

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
    yPID.setTolerance(5);
    addRequirements(hoodSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionMeasurement = NetworkTableInstance.getDefault().getTable("limelight-rrone").getEntry("ty").getDouble(0);
    visionSetpoint = (RangeLookup.getRangePair(RangeLookup.convertLLYtoRange(visionMeasurement))).getHoodAngle();
    hoodSubsystem.setHood(yPID.calculate(hoodSubsystem.getHoodPosition(), visionSetpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.setHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yPID.atSetpoint();
  }
}
