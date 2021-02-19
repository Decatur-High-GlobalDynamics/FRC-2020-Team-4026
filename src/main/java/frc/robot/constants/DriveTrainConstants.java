package frc.robot.constants;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;

public class DriveTrainConstants {
  // Following are made up. Update to reflect reality
  public static final double kDriveGearRatio = 10.71;
  public static final double kWheelRadiusMeters = 0.0762;
  public static final double kTrackWidthMeters = Units.inchesToMeters(19.5);

  public static final int kEncoderTicksPerRotation = 2048;
  public static final double kWheelCircumferenceMeters = 2 * Math.PI * kWheelRadiusMeters;

  // Distance per encoder pulse
  public static final double kEncoderDistancePerPulse =
      (2 * kWheelRadiusMeters * Math.PI) / (kEncoderTicksPerRotation * kDriveGearRatio);

  // Epsilon for DriveEncoders Command (in ticks)
  public static final double kDriveEpsilon = 50;

  public static final double kS = 0.22; // Static Volts
  public static final double kVLinear = 1.98; // Linear Velocity Volt Seconds per Meter
  public static final double KALinear = 0.2;  // Linear Acceleration Volt Seconds Squared per Meter
  public static final double kVAngular = 1.5; // Angular Velocity Volt Seconds per Meter 
  public static final double kAAngular = 0.3; // Angular Acceleration Volt Seconds Squared per Meter
  public static final DifferentialDriveKinematics kKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
  public static final DifferentialDriveVoltageConstraint kVoltageConstraint =
          new DifferentialDriveVoltageConstraint(
                  new SimpleMotorFeedforward(kS, kVLinear, KALinear),
                  kKinematics, 10); //10V max to account for battery sag
  public static final LinearSystem<N2, N2, N2> kPlant =
          LinearSystemId.identifyDrivetrainSystem(
                  kVLinear,
                  KALinear,
                  kVAngular,
                  kAAngular);
  public static final double kMaxSpeedMetersPerSecond = 3; // Tune
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
}
