// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other;

import frc.robot.Constants;

/** Add your docs here. */
public class RangeLookup  {
    public static int getHoodValue(double range){
        int hoodVal;
        switch(normalizeRange(range)){
            case 5:
            hoodVal = 0;
            break;
            case 10:
            hoodVal = 0;
            break;
            case 15:
            hoodVal = 0;
            break;
            case 20:
            hoodVal = 0;
            break;
            case 25:
            hoodVal = 0;
            break;
            case 30:
            hoodVal = 0;
            break;
            case 35:
            hoodVal = 0;
            break;
            case 40:
            hoodVal = 0;
            break;
            case 45:
            hoodVal = 0;
            break;
            default:
            hoodVal = 0;
            break;
        }
        return hoodVal;
    }


    private static int normalizeRange(double range){
        return (int) (5*(Math.round(range/5)));
    }

    public static double convertLLYtoRange(double ty){
        return (Constants.subsystems.hood.HubHeight - Constants.subsystems.hood.LLHeight) / Math.tan(Constants.subsystems.hood.LLAng + ty);
    }
}
