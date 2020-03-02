/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Utils {
    public static boolean checkTolerance(double valueA, double valueB, double epsilon ){
        if (Math.abs(valueA-valueB) <= epsilon)
            return true;
        else {
            return false;
        }
    }
}
