// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other;

/** Add your docs here. */
public abstract class RangePair {
    int hoodAng;

    public RangePair(int hoodAng) {
     
        this.hoodAng = hoodAng;
    }


    public int getHoodAngle() {
        return hoodAng;
    }
}

