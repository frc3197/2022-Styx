// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonFX intakeMotor;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
   
    intakeMotor = new WPI_TalonFX(Constants.subsystems.intake.intakeMotorID);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    //REVERSE IS UP!!!


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void useIntake(double val){intakeMotor.set(val);}
  
}
