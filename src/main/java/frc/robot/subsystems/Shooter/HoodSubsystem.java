// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Shooter.RangeLookup;
import frc.robot.other.extra_libraries.PIDConst;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private CANSparkMax hoodMotor;
  private static CANCoder encoder;
  PIDController yPID;
  PIDConst yPID_Constants;
  private SparkMaxLimitSwitch backLimit;

  // Back is reverse!!!
  public HoodSubsystem() {
    hoodMotor = new CANSparkMax(Constants.subsystems.hood.hoodMotorID, MotorType.kBrushless);
    hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    hoodMotor.setIdleMode(IdleMode.kBrake);
    hoodMotor.setInverted(false);
    
  yPID_Constants = Constants.subsystems.swerve.yALIGN_PID;
  yPID = new PIDController(yPID_Constants.p, yPID_Constants.i, yPID_Constants.d);
  yPID.setTolerance(5);
    encoder = new CANCoder(Constants.subsystems.hood.hoodEncoderID);
    backLimit = hoodMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Hood Aligned", yPID.atSetpoint());
    SmartDashboard.putNumber("Encoder Value Hood", encoder.getPosition());
    SmartDashboard.putBoolean("Back Limit Pressed", getHoodBackLimit());
    SmartDashboard.putNumber("Raw Range", RangeLookup
        .convertLLYtoRange(NetworkTableInstance.getDefault().getTable("limelight-rrone").getEntry("ty").getDouble(0)));
    SmartDashboard.putNumber("Normalized Range", RangeLookup.normalizeRange(RangeLookup
    .convertLLYtoRange(NetworkTableInstance.getDefault().getTable("limelight-rrone").getEntry("ty").getDouble(0))));
    // This method will be called once per scheduler run
  }

  /**
   * @param mode
   * @param x
   */
  public void setHood(double x) {
    hoodMotor.set(x);
  }

  public boolean getHoodBackLimit() {
    return backLimit.isPressed();

  }

  /**
   * Returns the position of the hood in raw sensor units.
   * 
   * @return double
   */
  public void setEncoderVal(int value) {
    encoder.setPosition(value);
  }
  public PIDController getPIDController(){
    return yPID;
  }
  public double getHoodPosition() {
    return encoder.getPosition();
  }

  public static void resetHoodEncoder() {
    encoder.setPosition(0);
  }

}
