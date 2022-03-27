// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other.extra_libraries;

/** Add your docs here. */
public class VoltagePair extends RangePair{
    double voltage;

    public VoltagePair(int hoodAng,double voltage) {
        super(hoodAng);
        this.voltage = voltage;
        //TODO Auto-generated constructor stub
    }
    public double getVoltage(){
        return voltage;
    }
    public int returnHoodAngle(){
        return super.getHoodAngle();
    }

}

