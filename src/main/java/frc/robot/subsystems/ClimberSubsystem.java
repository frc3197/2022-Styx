// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax spoolMotorLeft, spoolMotorRight;
  private int AFL_Limit, AFR_Limit, ABL_Limit, ABR_Limit;
  private SparkMaxLimitSwitch SUL_Limit, SUR_Limit, SLL_Limit, SLR_Limit;

  // TODO: Write Climber Subsystem
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    spoolMotorLeft = new CANSparkMax(Constants.subsystems.climber.spoolMotorLeftID, MotorType.kBrushless);
    spoolMotorRight = new CANSparkMax(Constants.subsystems.climber.spoolMotorRightID, MotorType.kBrushless);
    spoolMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0,100);
    spoolMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0,100);
    

    spoolMotorLeft.setIdleMode(IdleMode.kBrake);
    spoolMotorRight.setIdleMode(IdleMode.kBrake);
    // TODO:CONVERT

    SUL_Limit = spoolMotorLeft.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    SUR_Limit = spoolMotorRight.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    SLL_Limit = spoolMotorLeft.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    SLR_Limit = spoolMotorRight.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);  
  }

  @Override
  public void periodic() {
   // SmartDashboard.putBoolean("Spool Lower Limit Right", getSLR_Limit());
   // SmartDashboard.putBoolean("Spool Lower Limit Left", getSLL_Limit());
   // SmartDashboard.putBoolean("Spool Upper Limit Right", getSUR_Limit());
   // SmartDashboard.putBoolean("Spool Upper Limit Left", getSUL_Limit());
    
    // This method will be called once per scheduler run
  }


  public boolean getSLR_Limit() {
    return SLR_Limit.isPressed();
  }

  public boolean getSLL_Limit() {
    return SLL_Limit.isPressed();
  }

  public boolean getSUR_Limit() {
    return SUR_Limit.isPressed();
  }

  public boolean getSUL_Limit() {
    return SUL_Limit.isPressed();
  }



  public boolean getSL_Limits() {
    return getSLL_Limit();
  }

  public boolean getSU_Limits() {
    return getSUL_Limit();
  }


  public void setSpoolSpeed(double speed) {
    spoolMotorLeft.set(speed);
    spoolMotorRight.set(speed);

  }

}
