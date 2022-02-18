// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public enum ClimbType {
    high(0, 0, 0, 0), transversal(0, 0, 0, 0);

    public int unhook;
    public int hook;
    public int fTicks;
    public int waitTime;

    ClimbType(int unhook, int fTicks, int hook, int waitTime) {
        this.unhook = unhook;
        this.fTicks = fTicks;
        this.hook = hook;
        this.waitTime = waitTime;
    }
}
