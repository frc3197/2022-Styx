// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax spoolMotorLeft,spoolMotorRight;
  private WPI_TalonFX armMotorLeft, armMotorRight;
  private DutyCycleEncoder armEncoderLeft, armEncoderRight;
  //TODO: Write Climber Subsystem
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    spoolMotorLeft = new CANSparkMax(Constants.subsystems.climber.spoolMotorLeftID, MotorType.kBrushless);
    spoolMotorRight = new CANSparkMax(Constants.subsystems.climber.spoolMotorRightID,MotorType.kBrushless);
    armMotorLeft = new WPI_TalonFX(Constants.subsystems.climber.armMotorLeftID);
    armMotorRight = new WPI_TalonFX(Constants.subsystems.climber.armMotorRightID);
    armEncoderLeft = new DutyCycleEncoder(Constants.subsystems.climber.armEncoderLeftID);
    spoolMotorRight.follow(spoolMotorLeft);
    armMotorRight.follow(armMotorLeft);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSpoolMotor(double output){
    spoolMotorLeft.set(output);
  }
  public void setarmMotor(double output){
    armMotorLeft.set(output);
  }
  public double getEncoderValue(){
    return armEncoderLeft.get();
  }


}
