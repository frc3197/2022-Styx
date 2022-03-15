// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Add your docs here. */
public class RangeLookup {

    public static RangePair getRangePair(double range) {
        RangePair rangePair;
        switch (normalizeRange(range)) {
            case 70:
                rangePair = new VoltagePair(-480, .1);
                break;
            case 75:
            rangePair = new VoltagePair(-480, 2);
                break;
            case 80:
                rangePair = new RPMPair(-430, 1500);
                break;
            case 85:
            rangePair = new RPMPair(-450, 1500);
            break;
            case 90:
                rangePair = new RPMPair(-470, 1500);
                break;
            // VERIFY THIS RANGE 
            case 95:
            rangePair = new RPMPair(-465, 3500);
            break;
            case 100:
                rangePair = new RPMPair(-500, 1600);
                break;
            case 105:
            rangePair = new RPMPair(-520, 1500);
            break;
            case 110:
                rangePair = new RPMPair(-540, 3900);
                break;
                
            case 115:
            rangePair = new RPMPair(-600, 3900);
            break;
            case 120:
                rangePair = new RPMPair(-670, 4300);
                break;
                case 125:
                rangePair = new RPMPair(-675, 4300);
                break;
            case 130:
                rangePair = new RPMPair(-680, 4300);
                break;
                
            case 135:
            rangePair = new RPMPair(-700, 4350);
            break;
            case 140:
                rangePair = new RPMPair(0, 4400);
                break;
                
            case 145:
            rangePair = new RPMPair(-790, 4800);
            break;
            case 150:
                rangePair = new RPMPair(-850, 5100);
                break;
                case 155:
                rangePair = new RPMPair(-820, 5200);
                break;
            case 160:
                rangePair = new RPMPair(-780, 5300);
                break;
                case 165:
                rangePair = new RPMPair(-820, 5700);
                break;
        
            case 170:
                rangePair = new RPMPair(-850, 6200);
                break;
            case 175:
                rangePair = new RPMPair(-850, 6500);
                break;
            case 180:
                rangePair = new RPMPair(-850, 6500);
                break;
                case 185:
                rangePair = new RPMPair(0, 5800);
                break;
            case 190:
                rangePair = new RPMPair(0, 5800);
                break;
            case 200:
                rangePair = new RPMPair(0, 5800);
                break;
            case 210:
                rangePair = new RPMPair(0, 5800);
                break;
            case 220:
                rangePair = new RPMPair(0, 5800);
                break;
            case 230:
                rangePair = new RPMPair(0, 5800);
                break;
            case 240:
                rangePair = new RPMPair(0, 5800);
                break;
            case 250:
                rangePair = new RPMPair(0, 5800);
                break;
            case 260:
                rangePair = new RPMPair(0, 5800);
                break;
            case 270:
                rangePair = new RPMPair(0, 5800);
                break;
            case 280:
                rangePair = new RPMPair(0, 5800);
                break;
            case 290:
                rangePair = new RPMPair(0, 5800);
                break;
            default:
                rangePair = new RPMPair(0, 2500);
                break;
        }
        return rangePair;
    }

    public static int normalizeRange(double range) {
        return (int) (5 * (Math.round(range / 5)));
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
