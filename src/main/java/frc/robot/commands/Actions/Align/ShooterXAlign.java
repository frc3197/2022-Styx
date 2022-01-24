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
import frc.robot.subsystems.DriveSubsystem;

public class ShooterXAlign extends CommandBase {
  DriveSubsystem driveSubsystem;
  PIDController xPID;
  PIDConst xPID_Constants;
  ChassisSpeeds curSpeeds,newSpeeds;
  double visionSetpoint, visionMeasurement;

  /** Creates a new ShooterXAlign. */
  public ShooterXAlign(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    xPID_Constants = Constants.subsystems.swerve.xALIGN_PID;
    xPID = new PIDController(xPID_Constants.p, xPID_Constants.i, xPID_Constants.d);
    
    //TODO: TEST IF ADDREQUIREMENTS() IS NEEDED?
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curSpeeds = driveSubsystem.getChassisSpeeds();
    //TODO: FIX VISION ONCE LIMELIGHT IS UPDATED
    visionMeasurement = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    visionSetpoint = 0;
    newSpeeds = new ChassisSpeeds(curSpeeds.vxMetersPerSecond,curSpeeds.vyMetersPerSecond,xPID.calculate(visionMeasurement, visionSetpoint));
    driveSubsystem.drive(newSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xPID.atSetpoint();
  }
}
