// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Continuous;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.other.RangeLookup;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class Spool extends CommandBase {
  /** Creates a new AutoSpool. */
  private double rpm;
  ShooterSubsystem shooter;
  PIDController pid;
  double visionMeasurement;
  SimpleMotorFeedforward ff;
  LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  boolean inputExists;
  double input,ffOutput,pidOutput;
  public Spool(ShooterSubsystem shooter) {
    this.shooter = shooter;
    pid = new PIDController(Constants.subsystems.shooter.kP, Constants.subsystems.shooter.kI, Constants.subsystems.shooter.kD);
    ff = new SimpleMotorFeedforward(Constants.subsystems.shooter.kS, Constants.subsystems.shooter.kV, Constants.subsystems.shooter.kA);
    inputExists = false;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public Spool(ShooterSubsystem shooter, double input) {
    this.shooter = shooter;
    this.input = input;
    pid = new PIDController(Constants.subsystems.shooter.kP, Constants.subsystems.shooter.kI, Constants.subsystems.shooter.kD);
    ff = new SimpleMotorFeedforward(Constants.subsystems.shooter.kS, Constants.subsystems.shooter.kV, Constants.subsystems.shooter.kA);
    inputExists = true;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionMeasurement = NetworkTableInstance.getDefault().getTable("limelight-rrone").getEntry("ty").getDouble(0);
    rpm = RangeLookup.getRangePair(RangeLookup.convertLLYtoRange(visionMeasurement)).getRPM();
    if(inputExists){
       pidOutput = (pid.calculate(filter.calculate(shooter.getShooterRPM()), input));
       ffOutput = (ff.calculate(input));
    }
    else{
       pidOutput = (pid.calculate(filter.calculate(shooter.getShooterRPM()), rpm));
       ffOutput = (ff.calculate(rpm));
    }

    shooter.setVoltage((pidOutput + ffOutput) * Constants.subsystems.shooter.shooterMaxVoltage);

    if(pid.atSetpoint()){RobotContainer.getDriver2().setRumble(.8);}
    
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
