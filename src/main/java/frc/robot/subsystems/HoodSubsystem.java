// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.subsystems.hood;


public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private CANSparkMax hoodMotor;
  private CANCoder encoder;
  private SparkMaxLimitSwitch backLimit;  

  //Back is reverse!!!
  public HoodSubsystem() {
    hoodMotor = new CANSparkMax(Constants.subsystems.hood.hoodMotorID, MotorType.kBrushless);
    hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0,100);
    hoodMotor.setIdleMode(IdleMode.kBrake);
    hoodMotor.setInverted(false);
    encoder = new CANCoder(Constants.subsystems.hood.hoodEncoderID);
    backLimit = hoodMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Value Hood", encoder.getPosition());
    SmartDashboard.putBoolean("Back Limit Pressed", getHoodBackLimit());
    // This method will be called once per scheduler run
  }
  
  /** 
   * @param mode
   * @param x
   */
  public void setHood(double x){
    hoodMotor.set(x);
  }
  

  public boolean getHoodBackLimit(){
    return backLimit.isPressed();

  }
  /** 
   * Returns the position of the hood in raw sensor units.
   * @return double
   */
  public void setHoodValue(int value){
    encoder.setPosition(value);
  }
  
  public double getHoodPosition(){
    return encoder.getPosition();
  }



}
