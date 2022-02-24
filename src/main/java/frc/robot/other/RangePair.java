// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other;
/** Add your docs here. */
public class RangePair {
    int hoodAng, rpm;

    public RangePair(int hoodAng, int rpm) {
        this.rpm = rpm;
        this.hoodAng = hoodAng;
    }
    public int getHoodAngle(){
        return hoodAng;
    }
    public int getRPM(){
        return rpm;
    }
}
