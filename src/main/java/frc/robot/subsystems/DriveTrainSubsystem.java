/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Objects;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.DriveTrainConstants;
import frc.robot.constants.Ports;
import frc.robot.ITeamTalon;
import frc.robot.TeamTalonFX;
import frc.robot.TeamTalonSRX;

public class DriveTrainSubsystem extends SubsystemBase {
  private final DifferentialDrive drive;
  private final NavigationSubsystem nav;

  private static ITeamTalon rightDriveFalconMain;
  private static ITeamTalon leftDriveFalconMain;
  private static ITeamTalon rightDriveFalconSub;
  private static ITeamTalon leftDriveFalconSub;

  // Simulation stuff
  private DifferentialDrivetrainSim drivetrainSim;
  private final Field2d field = new Field2d();
  private double leftEncoderSimVelocity = 0, rightEncoderSimVelocity = 0;
  private double leftEncoderSimPosition = 0, rightEncoderSimPosition = 0;

  @SuppressWarnings("unused")
  private static TalonSRXSimCollection rightDriveFalconMainSim;

  @SuppressWarnings("unused")
  private static TalonSRXSimCollection leftDriveFalconMainSim;

  @SuppressWarnings("unused")
  private static TalonSRXSimCollection rightDriveFalconSubSim;

  @SuppressWarnings("unused")
  private static TalonSRXSimCollection leftDriveFalconSubSim;

  // This is a temp number that's theoretically best
  public double maxPowerChange = 0.215;
  public static double maxOutputSlow = .5;
  public static double maxOutputFast = 1;
  public double currentMaxPower = maxOutputSlow;
  public boolean rampingOn = true;

  private double velocityForStopMetersPerSecond = 0.2;

  public static enum DriveTrainMode {
    SLOW,
    FAST
  }

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
    throw new IllegalArgumentException(
        "not allowed! ctor must provide parameters for all dependencies");
  }

  public DriveTrainSubsystem(
      ITeamTalon rightDriveFalconMain,
      ITeamTalon leftDriveFalconMain,
      ITeamTalon rightDriveFalconSub,
      ITeamTalon leftDriveFalconSub,
      NavigationSubsystem nav) {
    rightDriveFalconMain =
        Objects.requireNonNull(rightDriveFalconMain, "rightDriveFalconMain must not be null");
    leftDriveFalconMain =
        Objects.requireNonNull(leftDriveFalconMain, "leftDriveFalconMain must not be null");
    rightDriveFalconSub =
        Objects.requireNonNull(rightDriveFalconSub, "rightDriveFalconSub must not be null");
    leftDriveFalconSub =
        Objects.requireNonNull(leftDriveFalconSub, "leftDriveFalconSub must not be null");
    this.nav = nav;

    if (RobotBase.isSimulation()) {
      rightDriveFalconMainSim = ((WPI_TalonSRX) rightDriveFalconMain).getSimCollection();
      leftDriveFalconMainSim = ((WPI_TalonSRX) rightDriveFalconMain).getSimCollection();
      rightDriveFalconSubSim = ((WPI_TalonSRX) rightDriveFalconMain).getSimCollection();
      leftDriveFalconSubSim = ((WPI_TalonSRX) rightDriveFalconMain).getSimCollection();
    }

    setupDrivetrain();
    drive = new DifferentialDrive(leftDriveFalconMain, rightDriveFalconMain);
    setupDifferentialDrive();

    if (RobotBase.isSimulation()) {
      drivetrainSim =
          new DifferentialDrivetrainSim(
              DriveTrainConstants.kPlant,
              DCMotor.getFalcon500(2),
              DriveTrainConstants.kDriveGearRatio,
              DriveTrainConstants.kTrackWidthMeters,
              DriveTrainConstants.kWheelRadiusMeters,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005) // Tune default STDEV?
              );
    }
  }

  private void configureTalon(ITeamTalon talon) {
    talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor(
        FeedbackDevice.IntegratedSensor, 0, 50); // use internal encoder
    talon.setSelectedSensorPosition(0, 0, 50); // zero encoders
    talon.config_kP(
        0, 0); // slotIdx, value  need to know the frc_characterization values to plug in here...
    talon.config_kI(0, 0);
    talon.config_kD(0, 0);
    talon.setNeutralMode(NeutralMode.Coast);
  }

  private void setupDrivetrain() {
    configureTalon(rightDriveFalconMain);
    configureTalon(leftDriveFalconMain);
    configureTalon(rightDriveFalconSub);
    configureTalon(leftDriveFalconSub);

    leftDriveFalconSub.follow(leftDriveFalconMain);
    rightDriveFalconSub.follow(rightDriveFalconMain);

    rightDriveFalconMain.setInverted(true);
    rightDriveFalconSub.setInverted(true);

    setDriveTrainMode(DriveTrainMode.SLOW);
    setBrakeMode(NeutralMode.Coast);
  }

  private void setupDifferentialDrive() {
    drive.setDeadband(0);
    drive.setRightSideInverted(true);
  }

  private void resetEncoders() {
    rightDriveFalconMain.setSelectedSensorPosition(0, 0, 50);
    leftDriveFalconMain.setSelectedSensorPosition(0, 0, 50);
    rightDriveFalconSub.setSelectedSensorPosition(0, 0, 50);
    leftDriveFalconSub.setSelectedSensorPosition(0, 0, 50);
  }

  public Pose2d getPose() {
    return nav.odometry.getPoseMeters();
  }

  public static DriveTrainSubsystem Create(NavigationSubsystem nav) {
    ITeamTalon rightDriveFalconMain;
    ITeamTalon leftDriveFalconMain;
    ITeamTalon rightDriveFalconSub;
    ITeamTalon leftDriveFalconSub;

    if (RobotBase.isSimulation()) {
      rightDriveFalconMain =
          new TeamTalonSRX("Subsystems.DriveTrain.RightMain", Ports.RightDriveFalconMainCAN);
      leftDriveFalconMain =
          new TeamTalonSRX("Subsystems.DriveTrain.LeftMain", Ports.RightDriveFalconMainCAN);
      rightDriveFalconSub =
          new TeamTalonSRX("Subsystems.DriveTrain.RightSub", Ports.RightDriveFalconMainCAN);
      leftDriveFalconSub =
          new TeamTalonSRX("Subsystems.DriveTrain.LeftSub", Ports.RightDriveFalconMainCAN);
    } else {
      rightDriveFalconMain =
          new TeamTalonFX("Subsystems.DriveTrain.RightMain", Ports.RightDriveFalconMainCAN);
      leftDriveFalconMain =
          new TeamTalonFX("Subsystems.DriveTrain.LeftMain", Ports.RightDriveFalconMainCAN);
      rightDriveFalconSub =
          new TeamTalonFX("Subsystems.DriveTrain.RightSub", Ports.RightDriveFalconMainCAN);
      leftDriveFalconSub =
          new TeamTalonFX("Subsystems.DriveTrain.LeftSub", Ports.RightDriveFalconMainCAN);
    }

    return new DriveTrainSubsystem(
        rightDriveFalconMain, leftDriveFalconMain, rightDriveFalconSub, leftDriveFalconSub, nav);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateMaxPowerChange();
    updateMaxDrivetrainOutputs();
    updateVelocityForStopMetersPerSecond();
    printDrivetrainData();
    updateMotorOutputs();
    field.setRobotPose(getPose());
    nav.odometry.update(
        Rotation2d.fromDegrees(nav.getHeading()),
        getLeftDistanceMeters(),
        getRightDistanceMeters());
    SmartDashboard.putNumber("Left position", getLeftPosition());
    SmartDashboard.putNumber("Right position", getRightPosition());
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.setInputs(
        leftDriveFalconMain.get() * RobotController.getBatteryVoltage(),
        rightDriveFalconMain.get() * RobotController.getBatteryVoltage());
    drivetrainSim.update(0.020);
    System.out.println("Gyro Set to: " + -drivetrainSim.getHeading().getDegrees());

    // Gyro
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // NavX expects clockwise positive, but sim outputs clockwise negative
    angle.set(-drivetrainSim.getHeading().getDegrees());

    // Encoders
    leftEncoderSimVelocity = metersToTicks(drivetrainSim.getLeftVelocityMetersPerSecond()) / 10d;
    leftEncoderSimPosition = metersToTicks(drivetrainSim.getLeftPositionMeters());
    rightEncoderSimVelocity = metersToTicks(drivetrainSim.getRightVelocityMetersPerSecond()) / 10d;
    rightEncoderSimPosition = metersToTicks(drivetrainSim.getRightPositionMeters());
  }

  public double ticksToRotations(double ticks) {
    return ticks / (double) DriveTrainConstants.kEncoderTicksPerRotation;
  }

  public double rotationsToTicks(double rotations) {
    return rotations * (double) DriveTrainConstants.kEncoderTicksPerRotation;
  }

  public double ticksToMeters(double ticks) {
    return ticksToRotations(ticks) * DriveTrainConstants.kWheelCircumferenceMeters;
  }

  public double metersToTicks(double meters) {
    double rotations = meters / DriveTrainConstants.kWheelCircumferenceMeters;
    return rotationsToTicks(rotations);
  }

  public double getLeftVelocityTicksPerDs() {
    if (RobotBase.isSimulation()) {
      return leftEncoderSimVelocity;
    }
    return (leftDriveFalconMain.getSelectedSensorVelocity(0)
            + leftDriveFalconSub.getSelectedSensorVelocity(0))
        / 2.0d;
  }

  public double getRightVelocityTicksPerDs() {
    if (RobotBase.isSimulation()) {
      return rightEncoderSimVelocity;
    }
    return (rightDriveFalconMain.getSelectedSensorVelocity(0)
            + rightDriveFalconSub.getSelectedSensorVelocity(0))
        / 2.0d;
  }

  private double getLeftPosition() {
    if (RobotBase.isSimulation()) {
      return leftEncoderSimPosition;
    }
    return (leftDriveFalconMain.getSelectedSensorPosition(0)
            + leftDriveFalconSub.getSelectedSensorPosition(0))
        / 2.0d;
  }

  private double getRightPosition() {
    if (RobotBase.isSimulation()) {
      return rightEncoderSimPosition;
    }
    return (rightDriveFalconMain.getSelectedSensorPosition(0)
            + rightDriveFalconSub.getSelectedSensorPosition(0))
        / 2.0d;
  }

  public double getLeftDistanceMeters() {
    double leftTicks = getLeftPosition();
    return ticksToMeters(leftTicks);
  }

  public double getRightDistanceMeters() {
    double rightTicks = getRightPosition();
    return ticksToMeters(rightTicks);
  }

  public void resetOdometry() {
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void resetOdometry(Pose2d translationPose) {
    resetEncoders();
    nav.odometry.resetPosition(translationPose, Rotation2d.fromDegrees(nav.getHeading()));
  }

  private void updateMaxPowerChange() {
    maxPowerChange =
        SmartDashboard.getNumber("Subsystems.DriveTrain.maxPowerChange", maxPowerChange);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxPowerChange", maxPowerChange);
  }

  private void updateMaxDrivetrainOutputs() {
    maxOutputSlow =
        SmartDashboard.getNumber("Subsystems.DriveTrain.maxPercentOutputSlow", maxOutputSlow);
    maxOutputFast =
        SmartDashboard.getNumber("Subsystems.DriveTrain.maxPercentOutputFast", maxOutputFast);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxOutputSlow", maxOutputSlow);
    SmartDashboard.putNumber("Subsystems.DriveTrain.maxOutputFast", maxOutputFast);
  }

  private void updateVelocityForStopMetersPerSecond() {
    velocityForStopMetersPerSecond =
        SmartDashboard.getNumber(
            "Subsystems.DriveTrain.velocityForStopMetersPerSecond", velocityForStopMetersPerSecond);
    SmartDashboard.putNumber(
        "Subsystems.DriveTrain.minimumSpeedForStopTicksPer100ms", velocityForStopMetersPerSecond);
  }

  private void printDrivetrainData() {
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPower", leftDriveFalconMain.get());
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPower", rightDriveFalconMain.get());
    // Print the power that's been demanded
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftPowerDemand", leftPowerSet);
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightPowerDemand", rightPowerSet);
  }

  private void updateMotorOutputs() {
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

  /**
   * caps the input power between -1 and +1
   *
   * @param desired
   * @return the capped power
   */
  private double getCappedPower(double desired) {
    return Math.max(Math.min(1, desired), -1);
  }

  // Caps the requested powers then sends them to Differential Drive
  public void setMotorPowers(double leftPowerDesired, double rightPowerDesired) {
    leftPowerDesired = getCappedPower(leftPowerDesired);
    rightPowerDesired = getCappedPower(rightPowerDesired);
    leftPowerSet = leftPowerDesired;
    rightPowerSet = rightPowerDesired;
  }

  public double getLeftEncoder() {
    return leftDriveFalconMain.getSelectedSensorPosition(0);
  }

  public double getRightEncoder() {
    return rightDriveFalconMain.getSelectedSensorPosition(0);
  }

  public void setDriveTrainMode(DriveTrainMode mode) {
    switch (mode) {
      case SLOW:
        currentMaxPower = maxOutputSlow;
        break;
      case FAST:
        currentMaxPower = maxOutputFast;
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
    return leftDriveFalconMain.getSelectedSensorVelocity(0)
            < speedInMetersToTicksPer100ms(velocityForStopMetersPerSecond)
        && rightDriveFalconMain.getSelectedSensorVelocity(0)
            < speedInMetersToTicksPer100ms(velocityForStopMetersPerSecond);
  }

  private int speedInMetersToTicksPer100ms(double speed) {
    return (int) Math.round(speed / (10 * DriveTrainConstants.kEncoderDistancePerPulse));
  }

  /* Never used...
  private double ticksPer100msToSpeedInMeters(int ticks) {
    return ticks * 10 * DriveTrainConstants.kEncoderDistancePerPulse;
  }
  */

  public void setRamping(boolean ramping) {
    rampingOn = ramping;
  }

  public void stop() {
    drive.tankDrive(0, 0);
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
