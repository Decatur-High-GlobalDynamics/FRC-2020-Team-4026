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

    //Define these
    public static final double ks = 0;
    public static final double kv = 0;
    public static final double ka = 0;
    //Define this
    public static final double kPDriveVel = 0;
    //make more accurate, was a bit rushed
    public static final double kTrackWidthMeters = 0.581025;
    public static final DifferentialDriveKinematics kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters);
    //TBD
    public static final double kMaxSpeedMetersPerSecond = 3;
    //TBD
    public static final double kMaxAccelMetersPerSecondSquared = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //TBD
    public static final double kDriveGearRatio = 1;

    public static final double kEncoderDistancePerPulse = (0.1524 * 2 * Math.PI * kDriveGearRatio) / 2048;

    // DIO Ports
    public static final int TurretLimitDIO = 0;

}
