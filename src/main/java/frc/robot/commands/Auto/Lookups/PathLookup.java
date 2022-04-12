// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Lookups;

import frc.robot.commands.Auto.PathContainer;

/** Add your docs here. */
public class PathLookup {

    private static double defaultTimeout = Double.POSITIVE_INFINITY;

    public static PathContainer getContainer(String pathName) {
        PathContainer ret = null;
        // PATH CONTAINER SYNTAX
        // new PathContainer(STRING PATH, DOUBLE ARRAY {MAX SPEED, MAX ACCELERATION},
        // DOUBLE TIMEOUT, BOOLEAN FIRST_PATH_IN AUTO)
        switch (pathName) {
            case "2BL1_1":
                ret = new PathContainer("2.1.1", getSpeeds(SPEEDS.THREE), 1.25, true, true);
                break;
            case "2BL1_F1":
                ret = new PathContainer("2.1.F1", getSpeeds(SPEEDS.TWO), defaultTimeout, false, true);
                break;
            case "2BL1_F2":
                ret = new PathContainer("2.1.F2", getSpeeds(SPEEDS.TWO), defaultTimeout, false, true);
                break;
            case "2BL2_1":
                ret = new PathContainer("2.2.1", getSpeeds(SPEEDS.FIVE), 2.5, true, true);
                break;
            case "2BL2_F1":
                ret = new PathContainer("2.2.F1", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BL2_F2":
                ret = new PathContainer("2.2.F2", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;

            case "2BL3_1":
                ret = new PathContainer("2.3.1", getSpeeds(SPEEDS.FIVE), 1.25, true, true);
                break;
            case "2BL3_F3":
                ret = new PathContainer("2.3.F3", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BL3_F4":
                ret = new PathContainer("2.3.F4", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;

            case "2BL4_1":
                ret = new PathContainer("2.4.1", getSpeeds(SPEEDS.FIVE), 1, true, true);
                break;
            case "2BL4_F3":
                ret = new PathContainer("2.4.F3", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BL4_F4":
                ret = new PathContainer("2.4.F4", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;

            case "3BL4_1":
                ret = new PathContainer("3.4.1", getSpeeds(SPEEDS.FIVE), defaultTimeout, true, true);
                break;
            case "3BL4_2":
                ret = new PathContainer("3.4.2", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "3BL4_F":
                ret = new PathContainer("3.4.F", getSpeeds(SPEEDS.FIVE), defaultTimeout, true, true);
                break;
            case "4BL3_1":
                ret = new PathContainer("4.3.1", getSpeeds(SPEEDS.FIVE), 1.25, true, true);
                break;
            case "4BL3_2":
                ret = new PathContainer("4.3.2", getSpeeds(SPEEDS.FIVE), 2.5, false, true);
                break;
            case "4BL3_3":
                ret = new PathContainer("4.3.3", getSpeeds(SPEEDS.FIVE), 1, false, true);
                break;
            case "4BL3_4":
                ret = new PathContainer("4.3.4", getSpeeds(SPEEDS.THREE), 2.5, false, true);
                break;
            case "4BL3_F":
                ret = new PathContainer("4.3.F", getSpeeds(SPEEDS.THREE), defaultTimeout, false, true);
                break;
            case "4BL4_1":
                ret = new PathContainer("4.4.1", getSpeeds(SPEEDS.FIVE), 1.25, true, true);
                break;
            case "4BL4_2":
                ret = new PathContainer("4.4.2", getSpeeds(SPEEDS.FIVE), 4.5, false, true);
                break;
            case "4BL4_3":
                ret = new PathContainer("4.4.3", getSpeeds(SPEEDS.FIVE), 2.5, false, true);
                break;
            case "4BL4_F":
                ret = new PathContainer("4.4.F", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "5BL4_1":
                ret = new PathContainer("5.4.1", getSpeeds(SPEEDS.FIVE), 1, true, true);
                break;
            case "5BL4_2":
                ret = new PathContainer("5.4.2", getSpeeds(SPEEDS.FIVE), 2, false, true);
                break;
            case "5BL4_3":
                ret = new PathContainer("5.4.3", getSpeeds(SPEEDS.FIVE), 3.5, false, true);
                break;
            case "5BL4_5":
                ret = new PathContainer("5.4.5", getSpeeds(SPEEDS.FIVE), 2.5, false, true);
                break;
                case "5BL4_F":
                ret = new PathContainer("5.4.F", getSpeeds(SPEEDS.FIVE), 2.5, false, true);
                break;
            case "2BFEN_1":
                ret = new PathContainer("2E_FEN.1.1", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BFEN_2":
                ret = new PathContainer("2E_FEN.1.2", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BFEN_3":
                ret = new PathContainer("2E_FEN.1.3", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BFEN_3A":
                ret = new PathContainer("2E_FEN.1.3 Alt", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BFEN_4":
                ret = new PathContainer("2E_FEN.4.", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BFEN_4A":
                ret = new PathContainer("2E_FEN.1.4 Alt", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BFEN_F":
                ret = new PathContainer("2E_FEN.1.F1", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BHAN_1":
                ret = new PathContainer("2E_HAN.1.1", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BHAN_2":
                ret = new PathContainer("2E_HAN.1.2", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BHAN1_3":
                ret = new PathContainer("2E_HAN1.1.3", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BHAN1_F":
                ret = new PathContainer("2E_HAN1.1.F", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BHAN2_3":
                ret = new PathContainer("2E_HAN2.1.3", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BHAN2_4":
                ret = new PathContainer("2E_HAN2.1.4", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;
            case "2BHAN2_F":
                ret = new PathContainer("2E_HAN2.1.F", getSpeeds(SPEEDS.FIVE), defaultTimeout, false, true);
                break;

        }
        return ret;
    }

    private enum SPEEDS {
        TWO, THREE, FOUR, FIVE
    }

    private static double[] getSpeeds(SPEEDS speeds) {
        double[] ret;
        switch (speeds) {
            case TWO:
                ret = new double[] { 2, 2 };
                break;
            case THREE:
                ret = new double[] { 3, 3 };
                break;
            case FOUR:
                ret = new double[] { 4, 4 };
                break;
            default:
            case FIVE:
                ret = new double[] { 5, 5 };
                break;
        }
        return ret;
    }
}
