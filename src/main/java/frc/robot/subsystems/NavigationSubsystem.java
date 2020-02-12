/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

public class NavigationSubsystem extends SubsystemBase {
    //private int VisionPI; //Change this to whatever type is needed

    DifferentialDriveOdometry odometry;

    DriveTrainSubsystem driveTrain;

    AHRS navx;

  /**
   * Creates a new NavigationSubsystem.
   */
  public NavigationSubsystem(DriveTrainSubsystem driveTrain) {
    //VisionPI = Constants.VisionPI;
    //Change this to whatever constructor is nescessary
    this.driveTrain = driveTrain;
    navx = new AHRS(SerialPort.Port.kMXP);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    //Change these to actual values
  }

  //Returns the robot's pose (position and rotation) in meters
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void calibratePose(Pose2d pose) {
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), driveTrain.getLeftEncoder(), driveTrain.getRightEncoder());
  }

  public double getHeading() {
    return navx.getYaw();
  }
}
