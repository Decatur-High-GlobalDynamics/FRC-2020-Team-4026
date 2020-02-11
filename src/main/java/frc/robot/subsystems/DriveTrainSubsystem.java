/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrainSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveTrainSubsystem.
   */
  final WPI_TalonFX rightDriveFalconMain;
  final WPI_TalonFX leftDriveFalconMain;
  final WPI_TalonFX rightDriveFalconSub;
  final WPI_TalonFX leftDriveFalconSub;

  public static final double defaultMaxPowerChange = 0.001;
  public static double maxPowerChange = defaultMaxPowerChange;
  public static final double basePowMod = .5;
  public static double powMod = basePowMod;


  public DriveTrainSubsystem() {
    rightDriveFalconMain = new WPI_TalonFX(Constants.RightDriveFalconMainCAN);
    leftDriveFalconMain = new WPI_TalonFX(Constants.LeftDriveFalconMainCAN);
    rightDriveFalconSub = new WPI_TalonFX(Constants.RightDriveFalconSubCAN);
    leftDriveFalconSub = new WPI_TalonFX(Constants.LeftDriveFalconSubCAN);

    rightDriveFalconMain.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftDriveFalconMain.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

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

  public void setMotorPowers(double rightPower, double leftPower){
    setRightMotorPower(rightPower);
    setLeftMotorPower(leftPower);
  }

  public void setRightMotorPower(double power){
    power = power * powMod;
    double curPower = rightDriveFalconMain.get();
    double nextPower;
    
    if (Math.abs(power - curPower) <= maxPowerChange){
      nextPower = power;
    } else {
      nextPower = curPower + Math.signum(power - curPower) * maxPowerChange;
    }

    rightDriveFalconMain.set(nextPower);
  }

  public void setLeftMotorPower(double power){
    power = power * powMod;
    double curPower = leftDriveFalconMain.get();
    double nextPower;
    if (Math.signum(power) == Math.signum(curPower) && Math.abs(power) < Math.abs(curPower)){
      nextPower = power;
    } else if (Math.abs(power - curPower) <= maxPowerChange){
      nextPower = power;
    } else {
      nextPower = curPower + Math.signum(power - curPower) * maxPowerChange;
    }

    leftDriveFalconMain.set(nextPower);
  }
}
