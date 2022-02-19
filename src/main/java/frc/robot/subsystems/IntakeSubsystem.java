// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  private WPI_TalonFX intakeMotor;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    armMotor = new CANSparkMax(Constants.subsystems.intake.armMotorID, MotorType.kBrushless);
    intakeMotor = new WPI_TalonFX(Constants.subsystems.intake.intakeMotorID);
    armMotor.setIdleMode(IdleMode.kBrake);
    //REVERSE IS UP!!!


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void useArm(double val){armMotor.set(val);}
  public void useIntake(double val){intakeMotor.set(val);}
  public CANSparkMax getArmMotor(){return armMotor;}
}
