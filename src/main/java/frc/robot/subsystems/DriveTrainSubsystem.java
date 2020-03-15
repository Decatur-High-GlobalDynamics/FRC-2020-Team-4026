/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  public double currentMaxPowerChange = maxPowerChange;
  public static double maxDrivetrainOutputSlowPercent = .5;
  public static double maxDrivetrainOutputFastPercent = 1;
  public double currentMaxDrivetrainOutputPercent = maxDrivetrainOutputSlowPercent;
  public boolean rampingOn = true;

  private boolean brakeMode = false;

  private double velocityForStopMetersPerSecond = 0.2;


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

    leftDriveFalconMain.setNeutralMode(NeutralMode.Coast);
    leftDriveFalconSub.setNeutralMode(NeutralMode.Coast);
    rightDriveFalconMain.setNeutralMode(NeutralMode.Coast);
    rightDriveFalconSub.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateMaxPowerChange();
    updateMaxDrivetrainOutputs();

    SmartDashboard.putNumber("Subsystems.DriveTrain.maxPowerChange", maxPowerChange);
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPower", leftDriveFalconMain.get());
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPower", rightDriveFalconMain.get());
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxOutputSlow", maxDrivetrainOutputSlowPercent);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxOutputFast", maxDrivetrainOutputFastPercent);
    velocityForStopMetersPerSecond = SmartDashboard.getNumber("Subsystems.DriveTrain.velocityForStopMetersPerSecond", velocityForStopMetersPerSecond);
    SmartDashboard.putNumber("Subsystems.DriveTrain.minimumSpeedForStopTicksPer100ms", velocityForStopMetersPerSecond);

   
  }

  private void updateMaxPowerChange() {
    maxPowerChange = SmartDashboard.getNumber("Subsystems.DriveTrain.maxPowerChange", maxPowerChange);
    
    if (rampingOn) currentMaxPowerChange = maxPowerChange;
    else currentMaxPowerChange = 1;
  }

  private void updateMaxDrivetrainOutputs() {
    maxDrivetrainOutputSlowPercent = SmartDashboard.getNumber("Subsystems.DriveTrain.maxPercentOutputSlow", maxDrivetrainOutputSlowPercent);
    maxDrivetrainOutputFastPercent = SmartDashboard.getNumber("Subsystems.DriveTrain.maxPercentOutputFast", maxDrivetrainOutputFastPercent);
  }

  //Caps the requested powers then sends them to Differential Drive
  public void setMotorPowers(double leftPowerDesired, double rightPowerDesired){
    leftPowerDesired = Math.max(Math.min(1, leftPowerDesired), -1);
    rightPowerDesired = Math.max(Math.min(1, rightPowerDesired), -1);
    //Display the power we are asking for
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPowerDemand", leftPowerDesired);
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPowerDemand", rightPowerDesired);
    leftPowerDesired *= currentMaxDrivetrainOutputPercent;
    rightPowerDesired *= currentMaxDrivetrainOutputPercent;

    //Divide by current max power bcause it was divided by it earlier, and that puts it back into the unit of "requested power", instead of "raw power", which is scaled by current max power
    double curRightPower = rightDriveFalconMain.get();
    double nextRightPower;
    if (Math.abs(rightPowerDesired - curRightPower) <= currentMaxPowerChange){
      nextRightPower = rightPowerDesired;
    } else {
      nextRightPower = curRightPower + Math.signum(rightPowerDesired - curRightPower) * currentMaxPowerChange;
    }

    double curleftPower = leftDriveFalconMain.get();
    double nextleftPower;
    if (Math.abs(leftPowerDesired - curleftPower) <= currentMaxPowerChange){
      nextleftPower = leftPowerDesired;
    } else {
      nextleftPower = curleftPower + Math.signum(leftPowerDesired - curleftPower) * currentMaxPowerChange;
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
    currentMaxDrivetrainOutputPercent = maxDrivetrainOutputFastPercent;
  }

  //sets it to half for controlability
  public void setSlowMode() {
    currentMaxDrivetrainOutputPercent = maxDrivetrainOutputSlowPercent;
  }

  public void setBrakeMode(NeutralMode mode) {
    leftDriveFalconMain.setNeutralMode(mode);
    leftDriveFalconSub.setNeutralMode(mode);
    rightDriveFalconMain.setNeutralMode(mode);
    rightDriveFalconSub.setNeutralMode(mode);
  }

  public boolean isStopped() {
    return leftDriveFalconMain.getSelectedSensorVelocity() < speedInMetersToTicksPer100ms(velocityForStopMetersPerSecond) 
    && rightDriveFalconMain.getSelectedSensorVelocity() < speedInMetersToTicksPer100ms(velocityForStopMetersPerSecond);
  }

  private int speedInMetersToTicksPer100ms(double speed) {
    return (int) Math.round(speed / (10 * Constants.kDriveEncoderDistancePerPulse));
  }

  private double ticksPer100msToSpeedInMeters(int ticks) {
    return ticks * 10 * Constants.kDriveEncoderDistancePerPulse;
  }

  public void enableRamping() {
    rampingOn = true;
  }

  public void disableRamping() {
    rampingOn = false;
  }
}
