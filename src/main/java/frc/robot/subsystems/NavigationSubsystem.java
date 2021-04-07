/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Objects;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

public class NavigationSubsystem extends SubsystemBase {
  DifferentialDriveOdometry odometry;
  AHRS navx;
  double accumulatedHeading = 0;
  double lastHeading = 0;

  public NavigationSubsystem() {
    throw new IllegalArgumentException(
        "not allowed! ctor must provide parameters for all dependencies");
  }

  public NavigationSubsystem(AHRS navx, DifferentialDriveOdometry odometry) {
    this.navx = Objects.requireNonNull(navx, "navx must not be null");
    this.odometry = Objects.requireNonNull(odometry, "odometry must not be null");
  }

  public static NavigationSubsystem Create() {
    AHRS navx = new AHRS(SerialPort.Port.kMXP);

    // This keeps a tally of position and heading and updates them based on encoders
    double originalHeading = navx.getYaw();
    DifferentialDriveOdometry odometry =
        new DifferentialDriveOdometry(Rotation2d.fromDegrees(originalHeading));

    return new NavigationSubsystem(navx, odometry);
  }

  // Returns the robot's pose (position and rotation) in meters
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // This updates the pose based on the encoder values, and heading. Not super accurate but good for
  // low time scales
  public void updatePoseNormally(double encoderLeft, double encoderRight) {
    odometry.update(Rotation2d.fromDegrees(getHeading()), encoderLeft, encoderRight);
  }

  // Calibrates the pose based on vision/lidar/etc.
  public void calibratePose(Pose2d pose) {
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Subsystems.Navigation.xCoord", getPose().getTranslation().getX());
    SmartDashboard.putNumber("Subsystems.Navigation.yCoord", getPose().getTranslation().getY());
    SmartDashboard.putNumber("Subsystems.Navigation.Heading", getPose().getRotation().getDegrees());

    double difference = 0;
    double currentHeading = getHeading();
    double currentHeadingTemp = currentHeading;
    if (Math.abs(currentHeading - lastHeading) > 270) {
      if (currentHeading < 0) {
        currentHeading = 360 - currentHeading;
      } else {
        currentHeading = currentHeading - 360;
      }
    }

    difference = currentHeading - lastHeading;
    lastHeading = currentHeadingTemp;

    accumulatedHeading += difference;
  }

  public double getHeading() {
    return navx.getYaw();
  }

  public double getAccumulatedHeading() {
    return accumulatedHeading;
  }

  public void resetHeading() {
    navx.reset();
  }
}
