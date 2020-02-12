/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public static final double defaultMaxPowerChange = 0.001;
  public static double maxPowerChange = defaultMaxPowerChange;
  public static final double basePowMod = .5;
  public static double powMod = basePowMod;


  public DriveTrainSubsystem() {
    rightDriveFalconMain = new WPI_TalonFX(Constants.RightDriveFalconMainCAN);
    leftDriveFalconMain = new WPI_TalonFX(Constants.LeftDriveFalconMainCAN);
    rightDriveFalconSub = new WPI_TalonFX(Constants.RightDriveFalconSubCAN);
    leftDriveFalconSub = new WPI_TalonFX(Constants.LeftDriveFalconSubCAN);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    rightDriveFalconMain.configAllSettings(configs);
    leftDriveFalconMain.configAllSettings(configs);

    SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftDriveFalconMain, leftDriveFalconSub);
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightDriveFalconMain, rightDriveFalconSub);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    leftDriveFalconSub.follow(leftDriveFalconMain);
    rightDriveFalconSub.follow(rightDriveFalconMain);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPower", leftDriveFalconMain.get());
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPower", rightDriveFalconMain.get());
    maxPowerChange = SmartDashboard.getNumber("Subsystems.DriveTrain.maxPowerChange", defaultMaxPowerChange);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxPowerChange", maxPowerChange);
    powMod = SmartDashboard.getNumber("Subsystems.DriveTrain.powMod", basePowMod);
    SmartDashboard.putNumber("Subsystems.DriveTrain.powMod", powMod);
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

  public int getLeftEncoder() {
    return leftDriveFalconMain.getSelectedSensorPosition();
  }

  public int getRightEncoder() {
    return rightDriveFalconMain.getSelectedSensorPosition();
  }

}
