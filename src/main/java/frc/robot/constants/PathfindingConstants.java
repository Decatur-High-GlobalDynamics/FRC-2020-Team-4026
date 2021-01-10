package frc.robot.constants;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class PathfindingConstants {
    //These are TBD: They will be used as feedforward for pathfinding
    public static final double ks = 0;
    public static final double kv = 0;
    public static final double ka = 0;
    //This is TBD: Will be a PID constant for pathfinding
    public static final double kPDriveVel = 0;
    //This is the width between our left and right wheels
    public static final double kTrackWidthMeters = 0.581025;
    //This uses that width to convert from chassis directions (power and angle) to powers for left and right wheels
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    //Our robots max speed. TBD
    public static final double kMaxSpeedMetersPerSecond = 3;
    //Our robots max acceleration. TBD
    public static final double kMaxAccelMetersPerSecondSquared = 3;

    //Gains for pathfinding. These are what WPI says is good for all bots
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}