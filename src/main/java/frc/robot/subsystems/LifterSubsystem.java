// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LifterSubsystem extends SubsystemBase {
  static DigitalInput upperBB;
  static DigitalInput lowerBB;
  WPI_TalonFX lowerFx, upperFx;
  boolean lowerMotorState, upperMotorState;
  double lowerSpeed, upperSpeed;
  /** Creates a new LifterSubsystem. */
  public LifterSubsystem() {
    lowerFx = new WPI_TalonFX(Constants.subsystems.lifter.lowerMotorID);
    lowerBB = new DigitalInput(Constants.subsystems.lifter.lowerBBChannel);
    upperFx = new WPI_TalonFX(Constants.subsystems.lifter.upperMotorID);
    upperBB = new DigitalInput(Constants.subsystems.lifter.upperBBChannel);

    lowerSpeed = Constants.subsystems.lifter.lowerSpeed;
    upperSpeed = Constants.subsystems.lifter.upperSpeed;
  }

  @Override
  public void periodic() {
    upperMotorState = (upperFx.get() != 0);
    lowerMotorState = (lowerFx.get() != 0);
    // This method will be called once per scheduler run
  }

  /**
   * @return boolean
   */
  public static boolean getLowerBB() {
    return lowerBB.get();
  }

  /**
   * @return boolean
   */
  public static boolean getUpperBB() {
    return upperBB.get();
  }

  
  /** 
   * @return double
   */
  public double getLowerSpeed() {
    return lowerSpeed;
  }

  
  /** 
   * @return double
   */
  public double getUpperSpeed() {
    return upperSpeed;
  }

  /**
   * @param liftSpeed
   */
  public void setLowerMotor(double liftSpeed) {
    lowerFx.set(liftSpeed);
  }

  /**
   * @param liftSpeed
   */
  public void setUpperMotor(double liftSpeed) {
    upperFx.set(liftSpeed);
  }

  /**
   * @param liftSpeed
   */
  public void setBothMotors(double liftSpeed) {
    setLowerMotor(liftSpeed);
    setUpperMotor(liftSpeed);
  }

  public void toggleLowerMotor() {
    setLowerMotor(lowerMotorState ? 0 : lowerSpeed);
  }

  public void toggleUpperMotor() {
    setUpperMotor(upperMotorState ? 0 : upperSpeed);
  }

  public void releaseBoth() {
    releaseUpper();
    releaseLower();
  }

  public void releaseUpper() {
    Timer.delay(.25);
    toggleUpperMotor();
  }

  public void releaseLower() {
    Timer.delay(.25);
    toggleLowerMotor();
  }

}
