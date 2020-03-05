/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

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

    public static void sendToNetworkTable(String tableName, String key, Object value) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
        if (table == null){
            return;
        }
        NetworkTableEntry entry = table.getEntry(key);
        if (entry == null){
            return;
        }
        entry.setValue(value);
    }

    public static Object getFromNetworkTable(String tableName, String key) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
        if (table == null){
            return null;
        }
        NetworkTableEntry entry = table.getEntry(key);
        if (entry == null){
            return null;
        }
        NetworkTableValue value = entry.getValue();
        if (value == null){
            return null;
        }
        return value.getValue();
    }
}
