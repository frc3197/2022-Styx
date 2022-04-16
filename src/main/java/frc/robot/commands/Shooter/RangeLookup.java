// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.other.extra_libraries.RPMPair;
import frc.robot.other.extra_libraries.RangePair;
import frc.robot.other.extra_libraries.VoltagePair;

/** Add your docs here. */
public class RangeLookup {
     

    public static RangePair getRangePair(double range) {
        RangePair rangePair;
        double voltage =7.4;
        int hoodAngle =-300;
        if(range<=175&&range>=70){
            voltage = (7.13+-0.0176*range+0.000129*(Math.pow(range, 2)));
            hoodAngle = ((int)(662+-6.97*range+0.0194*(Math.pow(range, 2))));
    }
    else if(range>175){voltage = 8;hoodAngle = 0;}

        rangePair = new VoltagePair(hoodAngle,voltage);
        
        return rangePair;
    }

    

    public static double convertLLYtoRange(double ty) {
        if (ty != 0) {
            return (Constants.subsystems.hood.HubHeight - Constants.subsystems.hood.LLHeight)
                    / Math.tan(Units.degreesToRadians(Constants.subsystems.hood.LLAng + ty));
        } else {
            return 0;
        }
    }
}
