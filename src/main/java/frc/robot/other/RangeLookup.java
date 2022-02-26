// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Add your docs here. */
public class RangeLookup {

    public static RangePair getRangePair(double range) {
        RangePair rangePair = new RangePair(0, 0);
        switch (normalizeRange(range)) {
            case 70:
                rangePair = new RangePair(-390, 3000);
                break;
            case 80:
                rangePair = new RangePair(-430, 3500);
                break;
            case 90:
                rangePair = new RangePair(-470, 3500);
                break;
            case 100:
                rangePair = new RangePair(-500, 3600);
                break;
            case 110:
                rangePair = new RangePair(-540, 3900);
                break;
            case 120:
                rangePair = new RangePair(-650, 4200);
                break;
            case 130:
                rangePair = new RangePair(-680, 4300);
                break;
            case 140:
                rangePair = new RangePair(-720, 4400);
                break;
            case 150:
                rangePair = new RangePair(-850, 5100);
                break;
            case 160:
                rangePair = new RangePair(-780, 5300);
                break;
                case 170:
                rangePair = new RangePair(-850, 6100);
                break;
        
                case 180:
                rangePair = new RangePair(0, 5800);
                break;
                case 190:
                rangePair = new RangePair(0, 5800);
                break;
                case 200:
                rangePair = new RangePair(0, 5800);
                break;
                case 210:
                rangePair = new RangePair(0, 5800);
                break;
                case 220:
                rangePair = new RangePair(0, 5800);
                break;
                case 230:
                rangePair = new RangePair(0, 5800);
                break;
                case 240:
                rangePair = new RangePair(0, 5800);
                break;
                case 250:
                rangePair = new RangePair(0, 5800);
                break;
                case 260:
                rangePair = new RangePair(0, 5800);
                break;
                case 270:
                rangePair = new RangePair(0, 5800);
                break;
                case 280:
                rangePair = new RangePair(0, 5800);
                break;
                case 290:
                rangePair = new RangePair(0, 5800);
                break;
            default:
                rangePair = new RangePair(0, 2500);
                break;
        }
        return rangePair;
    }

    public static int normalizeRange(double range) {
        return (int) (10 * (Math.round(range / 10)));
    }

    public static double convertLLYtoRange(double ty) {
        if(ty != 0){
        return (Constants.subsystems.hood.HubHeight - Constants.subsystems.hood.LLHeight)
                / Math.tan(Units.degreesToRadians(Constants.subsystems.hood.LLAng + ty));
    }
    else{
        return 0;
    }
}}
