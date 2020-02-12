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

    AHRS navx;

  /**
   * Creates a new NavigationSubsystem.
   */
  public NavigationSubsystem() {
    //VisionPI = Constants.VisionPI;
    //Change this to whatever constructor is nescessary
    navx = new AHRS(SerialPort.Port.kMXP);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    //Change these to actual values
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
    robotX = VisionPI.get(x);
    robotZ = VisionPI.get(z);
    robotHeading = VisionPI.get(heading);

    Change to whatever protocol is needed
    */
  }

  public double getHeading() {
    return navx.getYaw();
  }
}
