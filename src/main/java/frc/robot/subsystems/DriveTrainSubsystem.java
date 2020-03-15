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

  final WPI_TalonFX rightDriveFalconMain; 
  final WPI_TalonFX leftDriveFalconMain;
  final WPI_TalonFX rightDriveFalconSub;
  final WPI_TalonFX leftDriveFalconSub;
  //TBD: Make this a better value empirically
  public double maxPowerChange = 0.43;
  public static double maxDrivetrainOutputSlowPercent = .5;
  public static double maxDrivetrainOutputFastPercent = 1;
  public double currentMaxDrivetrainOutputPercent = maxDrivetrainOutputSlowPercent;
  public boolean rampingOn = true;

  private double velocityForStopMetersPerSecond = 0.2;

  public static enum DriveTrainMode {
    SLOW,
    FAST
  }


  public DriveTrainSubsystem() {
    rightDriveFalconMain = new WPI_TalonFX(Constants.RightDriveFalconMainCAN);
    leftDriveFalconMain = new WPI_TalonFX(Constants.LeftDriveFalconMainCAN);
    rightDriveFalconSub = new WPI_TalonFX(Constants.RightDriveFalconSubCAN);
    leftDriveFalconSub = new WPI_TalonFX(Constants.LeftDriveFalconSubCAN);
    setupDrivetrain();
    drive = new DifferentialDrive(leftDriveFalconMain, rightDriveFalconMain);
    setupDifferentialDrive();
  }

  private void setupDrivetrain() {
    //This configures the falcons to use their internal encoders
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    rightDriveFalconMain.configAllSettings(configs);
    leftDriveFalconMain.configAllSettings(configs);

    leftDriveFalconSub.follow(leftDriveFalconMain);
    rightDriveFalconSub.follow(rightDriveFalconMain);

    setDriveTrainMode(DriveTrainMode.SLOW);
    setBrakeMode(NeutralMode.Coast);
  }

  private void setupDifferentialDrive() {
    drive.setDeadband(0);
    drive.setRightSideInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateMaxPowerChange();
    updateMaxDrivetrainOutputs();
    updateVelocityForStopMetersPerSecond();
    printDrivetrainData();
  }

  private void updateMaxPowerChange() {
    maxPowerChange = SmartDashboard.getNumber("Subsystems.DriveTrain.maxPowerChange", maxPowerChange);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxPowerChange", maxPowerChange);
  }

  private void updateMaxDrivetrainOutputs() {
    maxDrivetrainOutputSlowPercent = SmartDashboard.getNumber("Subsystems.DriveTrain.maxPercentOutputSlow", maxDrivetrainOutputSlowPercent);
    maxDrivetrainOutputFastPercent = SmartDashboard.getNumber("Subsystems.DriveTrain.maxPercentOutputFast", maxDrivetrainOutputFastPercent);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxOutputSlow", maxDrivetrainOutputSlowPercent);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxOutputFast", maxDrivetrainOutputFastPercent);
  }

  private void updateVelocityForStopMetersPerSecond() {
    velocityForStopMetersPerSecond = SmartDashboard.getNumber("Subsystems.DriveTrain.velocityForStopMetersPerSecond", velocityForStopMetersPerSecond);
    SmartDashboard.putNumber("Subsystems.DriveTrain.minimumSpeedForStopTicksPer100ms", velocityForStopMetersPerSecond);
  }

  private void printDrivetrainData() {
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPower", leftDriveFalconMain.get());
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPower", rightDriveFalconMain.get());
  }

  //Caps the requested powers then sends them to Differential Drive
  public void setMotorPowers(double leftPowerDesired, double rightPowerDesired){
    leftPowerDesired = getCappedPower(leftPowerDesired);
    rightPowerDesired = getCappedPower(rightPowerDesired);
    //Display the power we are asking for
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPowerDemand", leftPowerDesired);
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPowerDemand", rightPowerDesired);
    leftPowerDesired *= currentMaxDrivetrainOutputPercent;
    rightPowerDesired *= currentMaxDrivetrainOutputPercent;

    
    double curRightPower = rightDriveFalconMain.get();
    double nextRightPower = rampingOn ? getRampingAdjustedPower(curRightPower, rightPowerDesired) : rightPowerDesired;
    

    double curleftPower = leftDriveFalconMain.get();
    double nextleftPower = rampingOn ? getRampingAdjustedPower(curleftPower, leftPowerDesired) : leftPowerDesired;

    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPowerGiven", nextRightPower);
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPowerGiven", nextleftPower);
    drive.tankDrive(nextleftPower, nextRightPower, false);
  }

   /**
   * Applies rampping logic to return the adjusted next power
   * @param currentPower
   * @param desired
   * @return adjusted power level
   */
  private double getRampingAdjustedPower(double currentPower, double desired){
    double rampped = desired;
    double requestedPowerChange = Math.abs(desired - currentPower);
    if (requestedPowerChange > maxPowerChange) {
      rampped = currentPower + Math.signum(desired - currentPower) * maxPowerChange;
    }
    return rampped;
  }

   /**
   * caps the input power between -1 and +1
   * @param desired
   * @return the capped power
   */
  private double getCappedPower(double desired) {
    return Math.max(Math.min(1, desired), -1);
  }

  public int getLeftEncoder() {
    return leftDriveFalconMain.getSelectedSensorPosition();
  }

  public int getRightEncoder() {
    return rightDriveFalconMain.getSelectedSensorPosition();
  }

  public void setDriveTrainMode(DriveTrainMode mode) {
    switch (mode) {
      case SLOW:
        currentMaxDrivetrainOutputPercent = maxDrivetrainOutputSlowPercent;
        break;
      case FAST:
        currentMaxDrivetrainOutputPercent = maxDrivetrainOutputFastPercent;
        break;
    }
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

  public void setRamping(boolean ramping) {
    rampingOn = ramping;
  }

  public void stop() {
    drive.tankDrive(0, 0);
  }
}
