// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Continuous;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class Spool extends CommandBase {
  /** Creates a new AutoSpool. */
  private double rpm;
  ShooterSubsystem shooter;
  PIDController pid;
  SimpleMotorFeedforward ff;
  public Spool(ShooterSubsystem shooter, double rpm) {
    this.rpm = rpm;
    this.shooter = shooter;
    pid = new PIDController(Constants.subsystems.shooter.kP, Constants.subsystems.shooter.kI, Constants.subsystems.shooter.kD);
    ff = new SimpleMotorFeedforward(Constants.subsystems.shooter.kS, Constants.subsystems.shooter.kV, Constants.subsystems.shooter.kA);
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double pidOutput = (pid.calculate(shooter.getShooterRPM(), rpm));
    double ffOutput = (ff.calculate(rpm));
    shooter.setVoltage((pidOutput + ffOutput) * Constants.subsystems.shooter.shooterMaxVoltage);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //NOTE: THIS SHOULD NOT END (USED IN RACE GROUP)
    return false;
  }
}
