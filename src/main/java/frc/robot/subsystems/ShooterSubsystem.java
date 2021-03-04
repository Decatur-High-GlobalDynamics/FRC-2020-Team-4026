/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Objects;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.constants.Ports;
import frc.robot.PidParameters;
import frc.robot.TeamSparkMAX;
import frc.robot.TeamUtils;

public class ShooterSubsystem extends SubsystemBase {
  private double maxRotationSpeedBot = 2226.5625;
  private double maxRotationSpeedTop = 1640.625;
  // Use 24500 for bot at init line

  private double targetSpeedBot = 585.9375;
  // Values for 95% (max power for shooting)
  private final TeamSparkMAX shooter_bottom;
  private final TeamSparkMAX shooter_top;
  /** Creates a new ShooterSubsystem. */
  private double shooterPowerTop = 0.95;

  private double shooterPowerBot = 0.95;

  private static double kPTop = 0.00005,
      kPBot = 0.00005,
      kITop = 0.000001,
      kIBot = 0.000001,
      kDTop = 0,
      kDBot = 0,
      kFTop = 0.0004,
      kFBot = 0.000156,
      kIZoneTop = 0,
      kIZoneBot = 0,
      kPeakOutputTop = 1,
      kPeakOutputBot = 1,
      maxVelTop = 2000,
      maxVelBot = 2000,
      maxAccTop = 1500,
      maxAccBot = 1500;
  private static int errorToleranceTop = 10, errorToleranceBot = 10;
  private PidParameters topPidParameters, botPidParameters;

  public ShooterSubsystem() {
    throw new IllegalArgumentException(
        "not allowed! ctor must provide parameters for all dependencies");
  }

  public ShooterSubsystem(
      TeamSparkMAX shooter_bottom,
      TeamSparkMAX shooter_top,
      PidParameters topPidParameters,
      PidParameters botPidParameters) {
    this.shooter_bottom = Objects.requireNonNull(shooter_bottom, "shooter_bottom must not be null");
    this.shooter_top = Objects.requireNonNull(shooter_top, "shooter_top must not be null");
    this.topPidParameters =
        Objects.requireNonNull(topPidParameters, "topPidParameters must not be null");
    this.botPidParameters =
        Objects.requireNonNull(botPidParameters, "botPidParameters must not be null");

    shooter_bottom.setInverted(true);
    shooter_top.setInverted(false);
    stop();
  }

  public static ShooterSubsystem Create() {
    TeamSparkMAX shooter_bottom =
        new TeamSparkMAX("Subsystems.Shooter.Bottom", Ports.BotShooterMotorCAN);
    TeamSparkMAX shooter_top = new TeamSparkMAX("Subsystems.Shooter.Top", Ports.TopShooterMotorCAN);
    PidParameters topPidParameters =
        new PidParameters(
            kPTop,
            kITop,
            kDTop,
            kFTop,
            kIZoneTop,
            kPeakOutputTop,
            maxVelTop,
            maxAccTop,
            errorToleranceTop);
    PidParameters botPidParameters =
        new PidParameters(
            kPBot,
            kIBot,
            kDBot,
            kFBot,
            kIZoneBot,
            kPeakOutputBot,
            maxVelBot,
            maxAccBot,
            errorToleranceBot);
    return new ShooterSubsystem(shooter_bottom, shooter_top, topPidParameters, botPidParameters);
  }

  @Override
  public void periodic() {
    shooter_bottom.periodic();
    shooter_top.periodic();
    topPidParameters.periodic("Subsystems.Shooter.Top", shooter_top, 0);
    botPidParameters.periodic("Subsystems.Shooter.Bot", shooter_bottom, 0);

    // This method will be called once per scheduler run
    shooterPowerTop =
        -Math.abs(SmartDashboard.getNumber("Subsystems.Shooter.shooterPowerTop", shooterPowerTop));
    SmartDashboard.putNumber("Subsystems.Shooter.shooterPowerTop", shooterPowerTop);
    shooterPowerBot =
        -Math.abs(SmartDashboard.getNumber("Subsystems.Shooter.shooterPowerBot", shooterPowerBot));
    SmartDashboard.putNumber("Subsystems.Shooter.shooterPowerBot", shooterPowerBot);

    SmartDashboard.putNumber("Subsystems.Shooter.maxRotationSpeedTop", maxRotationSpeedTop);
    SmartDashboard.putNumber("Subsystems.Shooter.maxRotaitonSpeedBot", maxRotationSpeedBot);

    /*
    targetSpeedTop = (int) SmartDashboard.getNumber("Subsystems.Shooter.targetSpeedTop", targetSpeedTop);
    SmartDashboard.putNumber("Subsystems.Shooter.targetSpeedTop", targetSpeedTop);
    */
    targetSpeedBot = SmartDashboard.getNumber("Subsystems.Shooter.targetSpeedBot", targetSpeedBot);
    SmartDashboard.putNumber("Subsystems.Shooter.targetSpeedBot", targetSpeedBot);

    SmartDashboard.putBoolean("Subsystems.Shooter.isShooterReady", this.isShooterReady());

    SmartDashboard.putNumber("Subsystems.Shooter.yAngleAdjusted", this.getVisionYAngle());
    SmartDashboard.putNumber("Subsystems.Shooter.knotDistance", this.getKnotDistance());

    kPTop = SmartDashboard.getNumber("Subsystems.Shooter.kPTop", kPTop);
    topPidParameters.kP = kPTop;
    SmartDashboard.putNumber("Subsystems.Shooter.kPTop", kPTop);
    kPBot = SmartDashboard.getNumber("Subsystems.Shooter.kPBot", kPBot);
    botPidParameters.kP = kPBot;
    SmartDashboard.putNumber("Subsystems.Shooter.kPBot", kPBot);
    kITop = SmartDashboard.getNumber("Subsystems.Shooter.kITop", kITop);
    topPidParameters.kI = kITop;
    SmartDashboard.putNumber("Subsystems.Shooter.kITop", kITop);
    kIBot = SmartDashboard.getNumber("Subsystems.Shooter.kIBot", kIBot);
    botPidParameters.kI = kIBot;
    SmartDashboard.putNumber("Subsystems.Shooter.kIBot", kIBot);
    kDTop = SmartDashboard.getNumber("Subsystems.Shooter.kDTop", kDTop);
    topPidParameters.kD = kDTop;
    SmartDashboard.putNumber("Subsystems.Shooter.kDTop", kDTop);
    kDBot = SmartDashboard.getNumber("Subsystems.Shooter.kDBot", kDBot);
    botPidParameters.kD = kDBot;
    SmartDashboard.putNumber("Subsystems.Shooter.kDBot", kDBot);
    kFTop = SmartDashboard.getNumber("Subsystems.Shooter.kFTop", kFTop);
    topPidParameters.kF = kFTop;
    SmartDashboard.putNumber("Subsystems.Shooter.kFTop", kFTop);
    kFBot = SmartDashboard.getNumber("Subsystems.Shooter.kFBot", kFBot);
    botPidParameters.kF = kFBot;
    SmartDashboard.putNumber("Subsystems.Shooter.kFBot", kFBot);
    kIZoneTop = SmartDashboard.getNumber("Subsystems.Shooter.kIZoneTop", kIZoneTop);
    topPidParameters.kIZone = kIZoneTop;
    SmartDashboard.putNumber("Subsystems.Shooter.kIZoneTop", kIZoneTop);
    kIZoneBot = SmartDashboard.getNumber("Subsystems.Shooter.kIZoneBot", kIZoneBot);
    botPidParameters.kIZone = kIZoneBot;
    SmartDashboard.putNumber("Subsystems.Shooter.kIZoneBot", kIZoneBot);
    kPeakOutputTop = SmartDashboard.getNumber("Subsystems.Shooter.kPeakOutputTop", kPeakOutputTop);
    topPidParameters.kPeakOutput = kPeakOutputTop;
    SmartDashboard.putNumber("Subsystems.Shooter.kPeakOutputTop", kPeakOutputTop);
    kPeakOutputBot = SmartDashboard.getNumber("Subsystems.Shooter.kPeakOutputBot", kPeakOutputBot);
    botPidParameters.kPeakOutput = kPeakOutputBot;
    /*SmartDashboard.putNumber("Subsystems.Shooter.kPeakOutputBot", kPeakOutputBot);
    maxVelTop = SmartDashboard.getNumber("Subsystems.Shooter.maxVelTop", maxVelTop);
    topPidParameters.maxVel = maxVelTop;
    SmartDashboard.putNumber("Subsystems.Shooter.maxVelTop", maxVelTop);
    maxVelBot = SmartDashboard.getNumber("Subsystems.Shooter.maxVelBot", maxVelBot);
    botPidParameters.maxVel = maxVelBot;
    SmartDashboard.putNumber("Subsystems.Shooter.maxVelBot", maxVelBot);
    kIZoneTop = SmartDashboard.getNumber("Subsystems.Shooter.maxAccTop", maxAccTop);
    topPidParameters.maxAcc = maxAccTop;
    SmartDashboard.putNumber("Subsystems.Shooter.maxAccTop", maxAccTop);
    maxAccBot = SmartDashboard.getNumber("Subsystems.Shooter.maxAccBot", maxAccBot);
    botPidParameters.maxAcc = maxAccBot;
    SmartDashboard.putNumber("Subsystems.Shooter.maxAccBot", maxAccBot);*/
    errorToleranceTop =
        (int) SmartDashboard.getNumber("Subsystems.Shooter.errorToleranceTop", errorToleranceTop);
    topPidParameters.errorTolerance = errorToleranceTop;
    SmartDashboard.putNumber("Subsystems.Shooter.errorToleranceTop", errorToleranceTop);
    errorToleranceBot =
        (int) SmartDashboard.getNumber("Subsystems.Shooter.errorToleranceBot", errorToleranceBot);
    botPidParameters.errorTolerance = errorToleranceBot;
    SmartDashboard.putNumber("Subsystems.Shooter.errorToleranceBot", errorToleranceBot);
  }

  public boolean isShooterReady() {
    return Math.abs(shooter_top.getVelocityError()) <= 600.0
        && Math.abs(shooter_bottom.getVelocityError()) <= 300.0;
  }

  public void setBottomMotor(double speed) {
    this.shooter_bottom.setSmartMotionVelocity(speed);
    // this.shooter_bottom.set(Math.abs(speed));
  }

  public void setTopMotor(double speed) {
    this.shooter_top.setSmartMotionVelocity(speed);
    // this.shooter_top.set(Math.abs(speed));
  }

  public double getShooterPowerTop() {
    return shooterPowerTop;
  }

  public double getShooterPowerBot() {
    return shooterPowerBot;
  }

  public void stop() {
    this.shooter_bottom.setSmartMotionVelocity(0);
    this.shooter_top.setSmartMotionVelocity(0);
    this.shooter_bottom.set(0);
    this.shooter_top.set(0);
  }

  public double getShooterSpeedTop() {
    return shooter_top.canEncoder.getVelocity();
  }

  public double getShooterSpeedBot() {
    return shooter_bottom.canEncoder.getVelocity();
  }

  public void setShooterVelTop(double speed) {
    speed = MathUtil.clamp(speed, 0, maxRotationSpeedTop);
    shooter_top.configureWithPidParameters(topPidParameters, 0);
    shooter_top.setSmartMotionVelocity(speed);
    // this.shooter_top.set(speed);
  }

  public void setShooterVelBot(double speed) {
    speed = MathUtil.clamp(speed, 0, maxRotationSpeedBot);
    shooter_bottom.configureWithPidParameters(botPidParameters, 0);
    shooter_bottom.setSmartMotionVelocity(speed);
    // this.shooter_bottom.set(speed);
  }

  public double getMaxVelTop() {
    return maxRotationSpeedTop;
  }

  public double getMaxVelBot() {
    return maxRotationSpeedBot;
  }

  public void setMotorVelocities(double topSpeedFraction, double botSpeedFraction) {
    setShooterVelTop((topSpeedFraction * maxRotationSpeedTop));
    setShooterVelBot((botSpeedFraction * maxRotationSpeedBot));
  }

  /*
  public int getTargetSpeedTop(){
    return targetSpeedTop;
  }
  */

  public double getTargetSpeedBot() {
    return targetSpeedBot;
  }

  public double getVisionYAngle() {
    Object result = TeamUtils.getFromNetworkTable("angles", "yAngle");
    if (result == null) {
      return 4026.0;
    } else {
      return (Double) result + 37.0;
    }
  }

  public double getKnotDistance() {
    return (1 / Math.tan(Math.toRadians(this.getVisionYAngle()))) * 54.5;
  }
}
