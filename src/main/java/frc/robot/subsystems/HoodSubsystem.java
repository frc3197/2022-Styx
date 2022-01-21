// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  WPI_TalonFX hoodMotor;
  public HoodSubsystem() {
    hoodMotor = new WPI_TalonFX(Constants.subsystems.hood.hoodMotorID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  /** 
   * @param mode
   * @param x
   */
  public void setHood(ControlMode mode, double x){
    hoodMotor.set(mode, x);
  }
  
  /** 
   * Returns the position of the hood in raw sensor units.
   * @return double
   */
  public double getHoodPosition(){
    return hoodMotor.getSelectedSensorPosition();
  }


}
