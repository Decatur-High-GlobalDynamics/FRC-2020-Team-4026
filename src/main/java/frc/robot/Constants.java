/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Placeholder Values
    public static final int LeftDriveFalconMainCAN = 1;
    public static final int LeftDriveFalconSubCAN = 2;
    public static final int RightDriveFalconMainCAN = 3;
    public static final int RightDriveFalconSubCAN =4;

    public static final int TopShooterMotorCAN = 5;
    public static final int BotShooterMotorCAN = 6;
    
    public static final int ClimbCAN = 7;

    public static final int IntakeCAN = 8;

    public static final int TurretCAN = 9;

    public static final int IndexerHorizPWM = 8;
    public static final int IndexerVertPWM = 9;

    //TBD
    public static final double ks = 0;
    public static final double kv = 0;
    public static final double ka = 0;
    //TBD
    public static final double kPDriveVel = 0;

    //Width between the wheels
    public static final double kTrackWidthMeters = 0.581025;
    //An object that can be used to go from chassis velocity (a speed and an angle) to velocities for the wheels
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    //Max speed for trajectory tracking
    public static final double kMaxSpeedMetersPerSecond = 4;
    //Max acceleration for trajectory tracking
    public static final double kMaxAccelMetersPerSecondSquared = 3;

    //These are gains for trajectory tracking, WPI said these gains were best for most bots
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //Gear ratio for drive. It is scaled DOWN by this much
    public static final double kDriveGearRatio = 10.71;

    public static final double kEncoderDistancePerPulse = (0.1524 * Math.PI) / (2048*kDriveGearRatio);

    // DIO Ports
    public static final int TurretLimitDIO = 0;

}
