package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class TeamUtils {
    public static double getCurrentTime() {
<<<<<<< HEAD
        return 1.0 * System.nanoTime()/1e9;
=======
        return 1.0 * System.nanoTime() / 1e9;
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
    public static boolean checkTolerance(double valueA, double valueB, double epsilon ){
        if (Math.abs(valueA-valueB) <= epsilon)
            return true;
        else {
            return false;
        }
>>>>>>> 005afeb5cd0205264ffb7badd989a03dab5b9bce
    }
}
