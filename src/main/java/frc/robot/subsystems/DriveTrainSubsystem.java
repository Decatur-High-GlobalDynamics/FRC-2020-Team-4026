/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.drivingCommands.TankDriveCommand;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class DriveTrainSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveTrainSubsystem.
   */
  final WPI_TalonFX rightDriveFalconMain;
  final WPI_TalonFX leftDriveFalconMain;
  final WPI_TalonFX rightDriveFalconSub;
  final WPI_TalonFX leftDriveFalconSub;

  final Encoder rightEncoder;
  final Encoder leftEncoder;

  public static final double maxPowerChange = 0.1;


  public DriveTrainSubsystem() {
    rightEncoder = new Encoder(Constants.RightEncoderCAN, Constants.RightEncoder2CAN, false);
    leftEncoder = new Encoder(Constants.LeftEncoderCAN, Constants.LeftEncoder2CAN, false);

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
  }

  public void setMotorPowers(double rightPower, double leftPower){
    setRightMotorPower(rightPower);
    setLeftMotorPower(leftPower);
  }

  public void setRightMotorPower(double power){
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
    double curPower = leftDriveFalconMain.get();
    double nextPower;
    
    if (Math.abs(power - curPower) <= maxPowerChange){
      nextPower = power;
    } else {
      nextPower = curPower + Math.signum(power - curPower) * maxPowerChange;
    }

    leftDriveFalconMain.set(nextPower);
  }
}
