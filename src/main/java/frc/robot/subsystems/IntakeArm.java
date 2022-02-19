// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {
  private CANSparkMax armMotor;

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    armMotor = new CANSparkMax(Constants.subsystems.intake.armMotorID, MotorType.kBrushless);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0,100);
    armMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public CANSparkMax getArmMotor(){return armMotor;}
  public void useArm(double val){armMotor.set(val);}

}
