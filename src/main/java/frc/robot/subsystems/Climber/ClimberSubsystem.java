// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonFX spoolMotorLeft, spoolMotorRight;
  private int SUL_Limit, SUR_Limit;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    spoolMotorLeft = new WPI_TalonFX(Constants.subsystems.climber.spoolMotorLeftID);
    spoolMotorRight = new WPI_TalonFX(Constants.subsystems.climber.spoolMotorRightID);
    
    spoolMotorLeft.setInverted(true);
    spoolMotorRight.setInverted(false);

    spoolMotorLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    spoolMotorRight.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    
    spoolMotorLeft.setNeutralMode(NeutralMode.Brake);
    spoolMotorRight.setNeutralMode(NeutralMode.Brake);
    //TODO: FIX

    SUL_Limit = spoolMotorLeft.getSensorCollection().isFwdLimitSwitchClosed();
    SUR_Limit = spoolMotorRight.getSensorCollection().isFwdLimitSwitchClosed();
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  public boolean getSUR_Limit() {
    return SUR_Limit == 1;
  }

  public boolean getSUL_Limit() {
    return SUL_Limit == 1;
  }

  public boolean getSU_Limits() {
    return getSUL_Limit();
  }


  public void setSpoolSpeed(double speed) {
    spoolMotorLeft.set(speed);
    spoolMotorRight.set(speed);

  }

}
