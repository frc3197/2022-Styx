// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArm extends SubsystemBase {
  private CANSparkMax armMotor;

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    armMotor = new CANSparkMax(Constants.subsystems.intake.armMotorID, MotorType.kBrushless);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0,255);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1,255);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2,255);

    
    armMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
  }
  public CANSparkMax getArmMotor(){return armMotor;}
  public void useArm(double val){armMotor.set(val);}

}
