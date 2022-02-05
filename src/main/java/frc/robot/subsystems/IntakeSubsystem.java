// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class IntakeSubsystem extends TrapezoidProfileSubsystem {

  CANSparkMax intakeMotor, armMotor;
  //TODO: Characterize Arm
   private static double kMaxVelRadPerSec = Constants.subsystems.intake.arm_maxVelRadPerSec;
   private static double kArmOffsetRadians = Constants.subsystems.intake.arm_offsetRad;
   private static double kMaxAccelRadPerSecSquared = Constants.subsystems.intake.arm_maxAccelRadPerSecSquared;
//TODO: LIMIT SWITCHES, no pid clean up intake
   private PIDController pid = Constants.subsystems.intake.armPIDController;
  ArmFeedforward feedforward = new ArmFeedforward(Constants.subsystems.intake.arm_kS, Constants.subsystems.intake.arm_kCos, Constants.subsystems.intake.arm_kV, Constants.subsystems.intake.arm_kA);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    super(
      // The constraints for the generated profiles
      new TrapezoidProfile.Constraints(kMaxVelRadPerSec, kMaxAccelRadPerSecSquared),
      // The initial position of the mechanism
      kArmOffsetRadians);
    intakeMotor = new CANSparkMax(Constants.subsystems.intake.intakeMotorID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    double ffOutput = feedforward.calculate(state.position, state.velocity);
    // Add the feedforward to the PID output to get the motor output
  
    armMotor.setVoltage(ffOutput);
  }

  public void setIntake(double speed){
    intakeMotor.set(speed);
  }
}
