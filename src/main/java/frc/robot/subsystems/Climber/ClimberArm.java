// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.DriveSubsystem;
@SuppressWarnings("unused")
public class ClimberArm extends SubsystemBase {
  private WPI_TalonFX armMotorLeft, armMotorRight;
  private int AFL_Limit, AFR_Limit, ABL_Limit, ABR_Limit;
  private static Encoder encoder;
  private ShuffleboardTab tab;
  

  /** Creates a new ClimberArm. */
  public ClimberArm() {
    armMotorLeft = new WPI_TalonFX(Constants.subsystems.climber.armMotorLeftID);
    armMotorRight = new WPI_TalonFX(Constants.subsystems.climber.armMotorRightID);
    encoder = new Encoder(Constants.subsystems.climber.armEncoderLeftA, Constants.subsystems.climber.armEncoderLeftB);
    armMotorRight.follow(armMotorLeft);

    armMotorLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
    armMotorRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);

    AFL_Limit = armMotorLeft.getSensorCollection().isFwdLimitSwitchClosed();
    AFR_Limit = armMotorRight.getSensorCollection().isFwdLimitSwitchClosed();
    ABL_Limit = armMotorLeft.getSensorCollection().isRevLimitSwitchClosed();
    ABR_Limit = armMotorRight.getSensorCollection().isRevLimitSwitchClosed();

    tab = Shuffleboard.getTab("Climber");

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder", getArmEncoderValueLeft());
    SmartDashboard.putNumber("Gyro Pitch", DriveSubsystem.getGyroscopeObj().getPitch());
    // This method will be called once per scheduler run
  }
  
  public boolean getAFL_Limit() {
    if (AFL_Limit == 0) {
      return false;
    } else {
      return true;
    }

  }

  public boolean getAFR_Limit() {
    if (AFR_Limit == 0) {
      return false;
    } else {
      return true;
    }
  }

  public boolean getABL_Limit() {
    if (ABL_Limit == 0) {
      return false;
    } else {
      return true;
    }

  }

  public boolean getABR_Limit() {
    if (ABR_Limit == 0) {
      return false;
    } else {
      return true;
    }
  }
    /**
  * @return boolean
  */
 public boolean getFrontLimits() {
   return getAFL_Limit() && getAFR_Limit();
 }

 /**
  * @return boolean
  */
 public boolean getBackLimits() {
   return getABL_Limit() && getABR_Limit();
 }
 
 public void setArmVoltage(double val) {
  armMotorLeft.setVoltage(val);
  
}

public void setArmSpeed(double speed) {
  armMotorLeft.set(speed);
  

}
public double getArmEncoderValueLeft(){
  return encoder.getDistance();
}
public static void resetEncoderValue(){
  encoder.reset();
}}
