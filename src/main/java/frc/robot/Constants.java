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
    //Port values for motors
    public static final int LeftDriveFalconMainCAN = 1;
    public static final int LeftDriveFalconSubCAN = 2;
    public static final int RightDriveFalconMainCAN = 3;
    public static final int RightDriveFalconSubCAN =4;

    public static final int TopShooterMotorCAN = 5;
    public static final int BotShooterMotorCAN = 6;
    
    public static final int LeftClimbCAN = 7;
    public static final int RightClimbCAN = 12;

    public static final int IntakeCAN = 8;

    public static final int TurretCAN = 9;

    public static final int IndexerHorizCAN = 10;
    public static final int IndexerVertCAN = 11;

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

    //Our gear ratio. TBD
    public static final double kDriveGearRatio = 10.71;

    //Distance per encoder pulse
    public static final double kEncoderDistancePerPulse = (0.1524 * Math.PI) / (2048 * kDriveGearRatio);

    //Epsilon for DriveEncoders Command (in ticks)
    public static final double driveEpsilon = 50;

    // DIO Ports
    public static final int TurretLimitDIO = 0;
    public static final int VerticalIndexer_BottomLimit_DIO = 1;
    public static final int VerticalIndexer_MiddleLimit_DIO = 2;
    public static final int VerticalIndexer_TopLimit_DIO = 3;


}
