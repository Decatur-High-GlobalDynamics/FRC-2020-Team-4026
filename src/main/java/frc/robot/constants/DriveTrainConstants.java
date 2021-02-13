package frc.robot.constants;

public class DriveTrainConstants {
  // Our gear ratio. TBD
  public static final double kDriveGearRatio = 10.71;

  // Distance per encoder pulse
  public static final double kEncoderDistancePerPulse =
      (0.1524 * Math.PI) / (2048 * kDriveGearRatio);

  // Epsilon for DriveEncoders Command (in ticks)
  public static final double driveEpsilon = 50;
}
