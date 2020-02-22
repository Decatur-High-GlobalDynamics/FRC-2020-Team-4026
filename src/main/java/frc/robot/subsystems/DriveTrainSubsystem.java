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
    double maxPowerChangeTemp = maxPowerChange;
    leftPowerDesired = Math.max(Math.min(1, leftPowerDesired), -1);
    rightPowerDesired = Math.max(Math.min(1, rightPowerDesired), -1);
    //Display the power we are asking for
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPowerDemand", leftPowerDesired);
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPowerDemand", rightPowerDesired);

    //Divide by current max power bcause it was divided by it earlier, and that puts it back into the unit of "requested power", instead of "raw power", which is scaled by current max power
    double curRightPower = rightDriveFalconMain.get()/currentMaxPower;
    double nextRightPower;
    if (Math.abs(rightPowerDesired - curRightPower) <= maxPowerChangeTemp){
      nextRightPower = rightPowerDesired;
    } else {
      nextRightPower = curRightPower + Math.signum(rightPowerDesired - curRightPower) * maxPowerChangeTemp;
    }

    double curleftPower = leftDriveFalconMain.get()/currentMaxPower;
    double nextleftPower;
    if (Math.abs(leftPowerDesired - curleftPower) <= maxPowerChangeTemp){
      nextleftPower = leftPowerDesired;
    } else {
      nextleftPower = curleftPower + Math.signum(leftPowerDesired - curleftPower) * maxPowerChangeTemp;
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

  //Sets the max output to full
  public void setFastMode() {
    currentMaxPower = maxOutputFast;
    drive.setMaxOutput(maxOutputFast);
  }

  //sets it to half for controlability
  public void setSlowMode() {
    currentMaxPower = maxOutputSlow;
    drive.setMaxOutput(maxOutputSlow);
  }

  public static double signPreservingSqrt(double input) {
    return Math.copySign(Math.sqrt(Math.abs(input)), input);
  }
}
