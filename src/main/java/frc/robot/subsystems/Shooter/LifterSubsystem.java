// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LifterSubsystem extends SubsystemBase {
  private static DigitalInput lifterBB;
  private static DigitalInput feederBB;
  private WPI_TalonFX lifterWheel;
  private CANSparkMax feederWheel;
  private boolean feederMotorState, lifterMotorState;
  private double feederSpeed, lifterSpeed;
  static boolean m_pressedLast = false;

  /** Creates a new LifterSubsystem. */
  public LifterSubsystem() {
    lifterWheel = new WPI_TalonFX(Constants.subsystems.lifter.lifterMotorID);
    lifterWheel.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
    lifterBB = new DigitalInput(Constants.subsystems.lifter.lifterBBChannel);
    feederWheel = new CANSparkMax(Constants.subsystems.lifter.feederMotorID, MotorType.kBrushless);
    feederWheel.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    feederBB = new DigitalInput(Constants.subsystems.lifter.feederBBChannel);
    feederSpeed = Constants.subsystems.lifter.feederSpeed;
    lifterSpeed = Constants.subsystems.lifter.lifterSpeed;

    lifterWheel.setInverted(false);
    lifterWheel.setNeutralMode(NeutralMode.Brake);
    feederWheel.setIdleMode(IdleMode.kBrake);
    feederWheel.setInverted(true);

  }

  @Override
  public void periodic() {
    lifterMotorState = (lifterWheel.get() != 0);
    feederMotorState = (feederWheel.get() != 0);
    SmartDashboard.putString("Lifter State", getLifterStateString());
    SmartDashboard.putBoolean("New Cargo", newCargo());
    // This method will be called once per scheduler run
  }

  /**
   * @return boolean
   */
  public static boolean getfeederBB() {
    return !feederBB.get();
  }

  public static boolean newCargo(){
    boolean result;
    boolean pressed = getfeederBB();

    if (!m_pressedLast && pressed) {
      result = true;
    }
    else{
      result = false;
    }
    m_pressedLast = pressed;
    return result;

  }

  /**
   * @return boolean
   */
  public static boolean getlifterBB() {
    return !lifterBB.get();
  }

  /**
   * @return double
   */
  public double getfeederSpeed() {
    return feederSpeed;
  }

  /**
   * @return double
   */
  public double getlifterSpeed() {
    return lifterSpeed;
  }

  /**
   * @param liftSpeed
   */
  public void setfeederMotor(double liftSpeed) {
    feederWheel.set(liftSpeed);
  }

  /**
   * @param liftSpeed
   */
  public void setfeederMotor(double liftSpeed, double delay) {
    Timer.delay(delay);
    feederWheel.set(liftSpeed);
  }

  /**
   * @param liftSpeed
   */
  public void setlifterMotor(double liftSpeed) {
    lifterWheel.set(liftSpeed);
  }

  /**
   * @param liftSpeed
   */
  public void setlifterMotor(double liftSpeed, double delay) {
    Timer.delay(delay);
    lifterWheel.set(liftSpeed);
  }

  /**
   * @param liftSpeed
   */
  public void setBothMotors(double liftSpeed) {
    setfeederMotor(liftSpeed);
    setlifterMotor(liftSpeed);
  }

  public void setBothMotors(double liftSpeed, double wait) {
    Timer.delay(wait);
    setfeederMotor(liftSpeed);
    setlifterMotor(liftSpeed);
  }

  public void toggleLowerMotor() {
    setfeederMotor(feederMotorState ? 0 : feederSpeed);
  }

  public void togglelifterMotor() {
    setlifterMotor(lifterMotorState ? 0 : lifterSpeed);
  }

  public void releaseBoth() {
    releaselifter();
    releaseLower();
  }

  public void releaselifter() {
    Timer.delay(.25);
    togglelifterMotor();
  }

  public void releaseLower() {
    Timer.delay(.25);
    toggleLowerMotor();
  }

  public void feed() {
    feederWheel.set(feederSpeed);
  }

  public void disableFeed() {
    feederWheel.set(0);
  }

  public static String getLifterStateString() {
    if (getfeederBB() && getlifterBB()) {
      return "2 Cargo in Lifter";
    } else if (getlifterBB() && !getfeederBB()) {
      return "1 Cargo in Lifter - Upper";
    } else if (getfeederBB() && !getlifterBB()) {
      return "1 Cargo in Lifter - Lower";
    } else {
      return "No Cargo in Lifter";
    }
  }
}
