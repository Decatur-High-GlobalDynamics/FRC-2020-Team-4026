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

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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
  //This was tested to be the lowest value where problems weren't had with the squaring thing that differential drive does
  public double maxPowerChange = 0.43;
  public static double maxOutputSlow = .5;
  public static double maxOutputFast = 1;
  public double currentMaxPower = maxOutputSlow;


  public DriveTrainSubsystem() {
    rightDriveFalconMain = new WPI_TalonFX(Constants.RightDriveFalconMainCAN);
    leftDriveFalconMain = new WPI_TalonFX(Constants.LeftDriveFalconMainCAN);
    rightDriveFalconSub = new WPI_TalonFX(Constants.RightDriveFalconSubCAN);
    leftDriveFalconSub = new WPI_TalonFX(Constants.LeftDriveFalconSubCAN);

    //This configures the falcons to use their internal encoders
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    rightDriveFalconMain.configAllSettings(configs);
    leftDriveFalconMain.configAllSettings(configs);

    leftDriveFalconSub.follow(leftDriveFalconMain);
    rightDriveFalconSub.follow(rightDriveFalconMain);


    //This wraps the motors
    drive = new DifferentialDrive(leftDriveFalconMain, rightDriveFalconMain);

    drive.setDeadband(0);
    
    setSlowMode();

    drive.setRightSideInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPower", leftDriveFalconMain.get());
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPower", rightDriveFalconMain.get());
    maxPowerChange = SmartDashboard.getNumber("Subsystems.DriveTrain.maxPowerChange", maxPowerChange);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxPowerChange", maxPowerChange);
    maxOutputSlow = SmartDashboard.getNumber("Subsystems.DriveTrain.maxOutputSlow", maxOutputSlow);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxOutputSlow", maxOutputSlow);
    maxOutputFast = SmartDashboard.getNumber("Subsystems.DriveTrain.maxOutputFast", maxOutputFast);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxOutputFast", maxOutputFast);
  }

  //Caps the requested powers then sends them to Differential Drive
  public void setMotorPowers(double leftPowerDesired, double rightPowerDesired){
    leftPowerDesired = Math.max(Math.min(1, leftPowerDesired), -1);
    rightPowerDesired = Math.max(Math.min(1, rightPowerDesired), -1);
    //Display the power we are asking for
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPowerDemand", leftPowerDesired);
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPowerDemand", rightPowerDesired);
    leftPowerDesired *= currentMaxPower;
    rightPowerDesired *= currentMaxPower;

    //Divide by current max power bcause it was divided by it earlier, and that puts it back into the unit of "requested power", instead of "raw power", which is scaled by current max power
    double curRightPower = rightDriveFalconMain.get();
    double nextRightPower;
    if (Math.abs(rightPowerDesired - curRightPower) <= maxPowerChange){
      nextRightPower = rightPowerDesired;
    } else {
      nextRightPower = curRightPower + Math.signum(rightPowerDesired - curRightPower) * maxPowerChange;
    }

    double curleftPower = leftDriveFalconMain.get();
    double nextleftPower;
    if (Math.abs(leftPowerDesired - curleftPower) <= maxPowerChange){
      nextleftPower = leftPowerDesired;
    } else {
      nextleftPower = curleftPower + Math.signum(leftPowerDesired - curleftPower) * maxPowerChange;
    }

    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPowerGiven", nextRightPower);
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPowerGiven", nextleftPower);
    drive.tankDrive(nextleftPower, nextRightPower, false);
  }

  public int getLeftEncoder() {
    return leftDriveFalconMain.getSelectedSensorPosition();
  }

  public int getRightEncoder() {
    return rightDriveFalconMain.getSelectedSensorPosition();
  }

  //This gets wheel speeds for pathing. The one issue is, the falcons get selected sensor velocity gives it in raw units per 100 ms, whereas the Wpi encoder gives as distance per
  //second, so I multiply by 10 then divide by distance per pulse. I don't know where to see how wpi does it so I can't say that multiplying by 10 is correct, but it should be
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds((leftDriveFalconMain.getSelectedSensorVelocity() * 10) / Constants.kEncoderDistancePerPulse, (rightDriveFalconMain.getSelectedSensorVelocity() * 10) * Constants.kEncoderDistancePerPulse);
  }

  public void tankDriveWithVolts(double leftVolts, double rightVolts) {
    rightDriveFalconMain.setVoltage(-rightVolts);
    leftDriveFalconMain.setVoltage(leftVolts);
    drive.feed();
  }

  //Sets the max output to full
  public void setFastMode() {
    currentMaxPower = maxOutputFast;
  }

  //sets it to half for controlability
  public void setSlowMode() {
    currentMaxPower = maxOutputSlow;
  }
}
