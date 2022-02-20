// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.subsystems.shooter;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private WPI_TalonFX shooterMotor;

  private Encoder shooterEncoder;
  // Note: Hood will most likely use built-in encoder

  public ShooterSubsystem() {
    shooterMotor = new WPI_TalonFX(Constants.subsystems.shooter.shooterMotorID);
    shooterEncoder = new Encoder(Constants.subsystems.shooter.shooterEncoderA, Constants.subsystems.shooter.shooterEncoderB);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM", getShooterRPM());
    // This method will be called once per scheduler run
  }

  
  /** 
   * @param mode
   * @param x
   */
  public void setShooter(ControlMode mode, double x) {
    shooterMotor.set(mode, x);
  }


  /** 
   * Returns the velocity of the shooter wheel in RPM.
   * @return double
   */
  public double getShooterRPM(){ 
    //TODO: VERIFY  
    return shooterEncoder.getRate() / 360 / 60 ;
  }

  public void setVoltage(double voltage)
  {
    shooterMotor.setVoltage(voltage);
  }
    

} 
