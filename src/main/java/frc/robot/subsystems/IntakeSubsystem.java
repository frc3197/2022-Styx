// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax armMotor, intakeMotor;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    armMotor = new CANSparkMax(Constants.subsystems.intake.armMotorID, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(Constants.subsystems.intake.intakeMotorID, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void useArm(double val){armMotor.set(val);}
  public void useIntake(double val){intakeMotor.set(val);}
}
