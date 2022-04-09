// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

/** Add your docs here. */
public class PathLookup {

    public static PathContainer getContainer(String pathName){
        PathContainer ret;
        // PATH CONTAINER SYNTAX
        // new PathContainer(STRING PATH, DOUBLE ARRAY {MAX SPEED, MAX ACCELERATION}, DOUBLE TIMEOUT)
        switch(pathName){
            default:
            case "a":
                ret = new PathContainer("A", new double[]{0,0}, 0, true);
            break;
        }   
        return ret;
    }
}
