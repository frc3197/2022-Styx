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
        switch (normalizeRange(range)) {
            case 70:
                rangePair = new VoltagePair(280, 6.4);
                break;
            case 75:
                rangePair = new VoltagePair(220, 6.5);
                break;
            case 80:
                rangePair = new VoltagePair(190, 6.6);
                break;
            case 85:
                rangePair = new VoltagePair(175, 6.6);
                break;
            case 90:
                rangePair = new VoltagePair(150, 6.625);
                break;
            // VERIFY THIS RANGE
            case 95:
                rangePair = new VoltagePair(125, 6.625);
                break;
            // Stopped Here
            case 100:
            rangePair = new VoltagePair(125, 6.65);
            break;
            case 105:
            rangePair = new VoltagePair(125, 6.7);
            break;
            case 110:
            rangePair = new VoltagePair(110, 6.75);
            break;

            case 115:
            rangePair = new VoltagePair(100, 6.8);
            break;
            case 120:
            rangePair = new VoltagePair(90, 6.8);
            break;
            case 125:
            rangePair = new VoltagePair(80, 7);
            break;
            case 130:
            rangePair = new VoltagePair(50, 7.1);
            break;

            case 135:
            rangePair = new VoltagePair(40, 7.3);
            break;
            case 140:
            rangePair = new VoltagePair(35, 7.3);
            break;

            case 145:
            rangePair = new VoltagePair(30, 7.35);
            break;
            case 150:
            rangePair = new VoltagePair(0, 7.4);
            break;
            case 155:
            rangePair = new VoltagePair(0, 7.5);
            break;
            case 160:
            rangePair = new VoltagePair(0, 7.6);
            break;
            case 165:
            rangePair = new VoltagePair(0, 7.65);
            break;

            case 170:
            rangePair = new VoltagePair(0, 7.9);
            break;
            case 175:
            rangePair = new VoltagePair(0, 8);
            break;
            case 180:
            rangePair = new VoltagePair(0, 7.8);
            break;
            case 185:
            rangePair = new VoltagePair(0, 7);
            break;
            case 190:
            rangePair = new VoltagePair(0, 7);
            break;
            case 200:
            rangePair = new VoltagePair(0, 7);
            break;
            case 210:
            rangePair = new VoltagePair(-710, 9.3);
                break;
            case 220:
            rangePair = new VoltagePair(-710, 9.5);
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
                rangePair = new VoltagePair(650, 6.8);
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
