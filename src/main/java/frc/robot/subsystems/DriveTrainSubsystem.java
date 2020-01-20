/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.drivingCommands.TankDriveCommand;


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
    rightEncoder = new Encoder(Constants.RightEncoderCAM, Constants.RightEncoder2CAM, false);
    leftEncoder = new Encoder(Constants.LeftEncoderCAM, Constants.LeftEncoder2CAM, false);

    rightDriveFalconMain = new WPI_TalonFX(Constants.RightDriveFalconMainCAM);
    leftDriveFalconMain = new WPI_TalonFX(Constants.LeftDriveFalconMainCAM);
    rightDriveFalconSub = new WPI_TalonFX(Constants.RightDriveFalconSubCAM);
    leftDriveFalconSub = new WPI_TalonFX(Constants.LeftDriveFalconSubCAM);

    leftDriveFalconSub.follow(leftDriveFalconMain);
    rightDriveFalconSub.follow(rightDriveFalconMain);

    //Sets the distance/encoder tick to be 1 unit. Should be updated after inches/encoeder is calculated. 
    leftEncoder.setDistancePerPulse(1);
    rightEncoder.setDistancePerPulse(1);

    setDefaultCommand(new TankDriveCommand(this));
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
