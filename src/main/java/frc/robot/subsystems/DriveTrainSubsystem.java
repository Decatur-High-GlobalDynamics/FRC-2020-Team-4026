/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;



public class DriveTrainSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveTrainSubsystem.
   */
  final DifferentialDrive drive;

  WPI_TalonFX rightDriveFalconMain; 
  WPI_TalonFX leftDriveFalconMain;
  WPI_TalonFX rightDriveFalconSub;
  WPI_TalonFX leftDriveFalconSub;

  DifferentialDriveOdometry odometry;

  public static final double maxPowerChange = 0.1;


  public DriveTrainSubsystem() {

    rightDriveFalconMain = new WPI_TalonFX(Constants.RightDriveFalconMainCAN);
    leftDriveFalconMain = new WPI_TalonFX(Constants.LeftDriveFalconMainCAN);
    rightDriveFalconSub = new WPI_TalonFX(Constants.RightDriveFalconSubCAN);
    leftDriveFalconSub = new WPI_TalonFX(Constants.LeftDriveFalconSubCAN);

    SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftDriveFalconMain, leftDriveFalconSub);
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightDriveFalconMain, rightDriveFalconSub);

    drive = new DifferentialDrive(leftMotors, rightMotors);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Caps the requested powers then sends them to Differential Drive
  public void setMotorPowers(double rightPower, double leftPower){
    double curRightPower = rightDriveFalconMain.get();
    double nextRightPower;
    
    if (Math.abs(rightPower - curRightPower) <= maxPowerChange){
      nextRightPower = rightPower;
    } else {
      nextRightPower = curRightPower + Math.signum(rightPower - curRightPower) * maxPowerChange;
    }

    double curleftPower = leftDriveFalconMain.get();
    double nextleftPower;
    
    if (Math.abs(leftPower - curleftPower) <= maxPowerChange){
      nextleftPower = leftPower;
    } else {
      nextleftPower = curleftPower + Math.signum(leftPower - curleftPower) * maxPowerChange;
    }

    
    drive.tankDrive(nextleftPower, nextRightPower);
  }

  //Returns the robot's pose (position and rotation) in meters
  public Pose2d getPost() {
    return odometry.getPoseMeters();
  }
}
