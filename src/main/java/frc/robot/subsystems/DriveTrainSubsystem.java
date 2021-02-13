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

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */
  final DifferentialDrive drive;

  WPI_TalonFX rightDriveFalconMain;
  WPI_TalonFX leftDriveFalconMain;
  WPI_TalonFX rightDriveFalconSub;
  WPI_TalonFX leftDriveFalconSub;
  // TODO: Fix max power change now that it's in units per second
  public double maxPowerChange = 0.43;
  public static double maxOutputSlow = .5;
  public static double maxOutputFast = 1;
  public double currentMaxPower = maxOutputSlow;
  public boolean rampingOn = true;

  private boolean brakeMode = false;

  private double epsilonIsStopped = 100;

  private SlewRateLimiter rightLimiter = new SlewRateLimiter(maxPowerChange);
  private SlewRateLimiter leftLimiter = new SlewRateLimiter(maxPowerChange);

  private double leftPowerSet = 0;
  private double rightPowerSet = 0;

  // These three are brought over from differential drive in order to bring over it's curvature
  // drive code
  private double m_quickStopAccumulator;
  private double m_quickStopThreshold;
  private double m_quickStopAlpha;

  public DriveTrainSubsystem() {
    rightDriveFalconMain = new WPI_TalonFX(Constants.RightDriveFalconMainCAN);
    leftDriveFalconMain = new WPI_TalonFX(Constants.LeftDriveFalconMainCAN);
    rightDriveFalconSub = new WPI_TalonFX(Constants.RightDriveFalconSubCAN);
    leftDriveFalconSub = new WPI_TalonFX(Constants.LeftDriveFalconSubCAN);

    // This configures the falcons to use their internal encoders
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    rightDriveFalconMain.configAllSettings(configs);
    leftDriveFalconMain.configAllSettings(configs);

    leftDriveFalconSub.follow(leftDriveFalconMain);
    rightDriveFalconSub.follow(rightDriveFalconMain);

    // This wraps the motors
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
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPower", leftDriveFalconMain.get());
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPower", rightDriveFalconMain.get());
    maxPowerChange =
        SmartDashboard.getNumber("Subsystems.DriveTrain.maxPowerChange", maxPowerChange);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxPowerChange", maxPowerChange);
    maxOutputSlow = SmartDashboard.getNumber("Subsystems.DriveTrain.maxOutputSlow", maxOutputSlow);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxOutputSlow", maxOutputSlow);
    maxOutputFast = SmartDashboard.getNumber("Subsystems.DriveTrain.maxOutputFast", maxOutputFast);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxOutputFast", maxOutputFast);
    epsilonIsStopped =
        SmartDashboard.getNumber("Subsystems.DriveTrain.epsilonIsStopped", epsilonIsStopped);
    SmartDashboard.putNumber("Subsystems.DriveTrain.epsilonIsStopped", epsilonIsStopped);
    // Print the power that's been demanded
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPowerDemand", leftPowerSet);
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPowerDemand", rightPowerSet);
    // Scale it by the current max power - so if we're not in fast mode, everything goes at half
    // speed
    double cappedLeftPowerDesired = leftPowerSet * currentMaxPower;
    double cappedRightPowerDesired = rightPowerSet * currentMaxPower;
    // Set up vars for putting the final power in - they need to be set up here because of scope
    // stuff
    double nextRightPower;
    double nextleftPower;
    // If you're ramping, use the calculate function on the limiter to calculate the next speed
    if (rampingOn) {
      nextleftPower = leftLimiter.calculate(cappedLeftPowerDesired);
      nextRightPower = rightLimiter.calculate(cappedRightPowerDesired);
    } else {
      // If you aren't ramping, just set your next power to whatever asked
      nextleftPower = cappedLeftPowerDesired;
      nextRightPower = cappedRightPowerDesired;
      // This is important - it ensures the limiters always keep up with the current speed. They
      // usually like to be called with calculate, but that's obviously not
      // possible when not ramping, so instead we just constantly force the limiters to catch up
      // with us. This means that whenever we start ramping again they'll be caught up
      rightLimiter.reset(nextRightPower);
      leftLimiter.reset(nextleftPower);
    }

    // Print the power that's going to the motors
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPowerGiven", nextRightPower);
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPowerGiven", nextleftPower);
    // Send it to the motors. The false at the end lets you not square the power - bc that leads to
    // weird ramping stuff
    drive.tankDrive(nextleftPower, nextRightPower, false);
  }

  // Caps the requested powers then sends them to Differential Drive
  public void setMotorPowers(double leftPowerDesired, double rightPowerDesired) {
    leftPowerDesired = Math.max(Math.min(1, leftPowerDesired), -1);
    rightPowerDesired = Math.max(Math.min(1, rightPowerDesired), -1);
    leftPowerSet = leftPowerDesired;
    rightPowerSet = rightPowerDesired;
  }

  public double getLeftEncoder() {
    return leftDriveFalconMain.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return rightDriveFalconMain.getSelectedSensorPosition();
  }

  // Sets the max output to full
  public void setFastMode() {
    currentMaxPower = maxOutputFast;
  }

  // sets it to half for controlability
  public void setSlowMode() {
    currentMaxPower = maxOutputSlow;
  }

  public void toggleBrakemode() {
    if (brakeMode) {
      leftDriveFalconMain.setNeutralMode(NeutralMode.Coast);
      leftDriveFalconSub.setNeutralMode(NeutralMode.Coast);
      rightDriveFalconMain.setNeutralMode(NeutralMode.Coast);
      rightDriveFalconSub.setNeutralMode(NeutralMode.Coast);
    }
    if (!brakeMode) {
      leftDriveFalconMain.setNeutralMode(NeutralMode.Brake);
      leftDriveFalconSub.setNeutralMode(NeutralMode.Brake);
      rightDriveFalconMain.setNeutralMode(NeutralMode.Brake);
      rightDriveFalconSub.setNeutralMode(NeutralMode.Brake);
    }
    brakeMode = !brakeMode;
  }

  public boolean isStopped() {
    return leftDriveFalconMain.getSelectedSensorVelocity() < 100
        && rightDriveFalconMain.getSelectedSensorVelocity() < 100;
  }

  public void enableRamping() {
    rampingOn = true;
  }

  public void disableRamping() {
    rampingOn = false;
  }

  // This is literally just the curvature drive mode from differntial drive - the differnce is that
  // it feeds it to our management system for motor power instead, which means ramping
  // and max power change apply
  public void curveDrive(double speed, double rotation, boolean turnInPlace) {
    speed = MathUtil.clamp(speed, -1.0, 1.0);

    rotation = MathUtil.clamp(rotation, -1.0, 1.0);

    double angularPower;
    boolean overPower;

    if (turnInPlace) {
      if (Math.abs(speed) < m_quickStopThreshold) {
        m_quickStopAccumulator =
            (1 - m_quickStopAlpha) * m_quickStopAccumulator
                + m_quickStopAlpha * MathUtil.clamp(rotation, -1.0, 1.0) * 2;
      }
      overPower = true;
      angularPower = rotation;
    } else {
      overPower = false;
      angularPower = Math.abs(speed) * rotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = speed + angularPower;
    double rightMotorOutput = speed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    setMotorPowers(leftMotorOutput, rightMotorOutput);
  }
}
