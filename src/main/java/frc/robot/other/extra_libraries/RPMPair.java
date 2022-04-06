// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other.extra_libraries;

/** Add your docs here. */
public class RPMPair extends RangePair{
    int rpm;

    public RPMPair(int hoodAng,int rpm) {
        super(hoodAng);
        this.rpm = rpm;
    }
    public int getRPM(){
        return rpm;
    }
    public int returnHoodAngle(){
        return super.getHoodAngle();
    }

}
