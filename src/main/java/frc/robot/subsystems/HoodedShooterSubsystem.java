// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.TeamSparkMAX;
import frc.robot.constants.Ports;

public class HoodedShooterSubsystem extends SubsystemBase {
  private final TeamSparkMAX shooter_left;
  private final TeamSparkMAX shooter_right;
  private final double maxVelLeft = 1;
  private final double maxVelRight = 1;
  /** Creates a new HoodedShooter. */
  public HoodedShooterSubsystem() {

    throw new IllegalArgumentException(
        "not allowed! ctor must provide parameters for all dependencies");
  }

  public HoodedShooterSubsystem(TeamSparkMAX shooter_left, TeamSparkMAX shooter_right) {
    this.shooter_left = Objects.requireNonNull(shooter_left, "shooter_bottom must not be null");
    this.shooter_right = Objects.requireNonNull(shooter_right, "shooter_top must not be null");

    shooter_left.setInverted(true);
    shooter_right.setInverted(false);
    stop();
  }

  public static HoodedShooterSubsystem Create() {
    TeamSparkMAX shooter_bottom =
        new TeamSparkMAX("Subsystems.Shooter.Left", Ports.LeftShooterMotorCAN);
    TeamSparkMAX shooter_top =
        new TeamSparkMAX("Subsystems.Shooter.Right", Ports.RightShooterMotorCAN);

    return new HoodedShooterSubsystem(shooter_bottom, shooter_top);
  }

  @Override
  public void periodic() {
    shooter_left.periodic();
    shooter_right.periodic();
  }

  public void setLeftMotor(double speed) {
    this.shooter_left.setSmartMotionVelocity(speed);
  }

  public void setRightMotor(double speed) {
    this.shooter_right.setSmartMotionVelocity(speed);
  }

  public double getShooterPowerLeft() {
    return shooter_left.get();
  }

  public double getShooterPowerRight() {
    return shooter_right.get();
  }

  public void stop() {
    this.shooter_left.setSmartMotionVelocity(0);
    this.shooter_right.setSmartMotionVelocity(0);
    this.shooter_left.set(0);
    this.shooter_right.set(0);
  }

  public double getShooterSpeedLeft() {
    return shooter_left.canEncoder.getVelocity();
  }

  public double getShooterSpeedRight() {
    return shooter_right.canEncoder.getVelocity();
  }

  public void setShooterVelLeft(double speed) {
    speed = MathUtil.clamp(speed, 0, maxVelLeft);
    shooter_left.setSmartMotionVelocity(speed);
    // this.shooter_bottom.set(speed);
  }

  public void setShooterVelRight(double speed) {
    speed = MathUtil.clamp(speed, 0, maxVelRight);
    shooter_right.setSmartMotionVelocity(speed);
    // this.shooter_top.set(speed);
  }

  public boolean isShooterReady() {
    return Math.abs(shooter_left.getVelocityError()) <= 600.0
        || Math.abs(shooter_right.getVelocityError()) <= 300.0;
  }

  public void setMotorVelocities(int leftSpeed, double rightSpeed) {
    setShooterVelLeft(leftSpeed);
    setShooterVelRight(rightSpeed);
  }
}
