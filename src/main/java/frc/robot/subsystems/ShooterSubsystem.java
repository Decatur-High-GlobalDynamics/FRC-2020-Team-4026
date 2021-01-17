/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.PidParameters;
import frc.robot.TeamTalonSRX;
import frc.robot.TeamUtils;

public class ShooterSubsystem extends SubsystemBase {
  private int maxRotationSpeedBot = 38000;
  private int maxRotationSpeedTop = 28000;
  // Use 24500 for bot at init line

  private int targetSpeedBot = 10000;
  // Values for 95% (max power for shooting)
  private final TeamTalonSRX shooter_bottom;
  private final TeamTalonSRX shooter_top;
  /** Creates a new ShooterSubsystem. */
  private double shooterPowerTop = 0.95;

  private double shooterPowerBot = 0.95;

  private PidParameters topPidParameters = new PidParameters(0.3, 0.00015, 0.1, 0.031, 0, 1, 10);
  private PidParameters botPidParameters = new PidParameters(0.1, 0.00005, 0.1, 0.026, 250, 1, 10);

  public ShooterSubsystem() {
    shooter_bottom = new TeamTalonSRX("Subsystems.Shooter.Bottom", Constants.BotShooterMotorCAN);
    shooter_top = new TeamTalonSRX("Subsystems.Shooter.Top", Constants.TopShooterMotorCAN);
    shooter_top.setSensorPhase(true);
    shooter_bottom.setSensorPhase(true);
    shooter_bottom.setInverted(true);
    shooter_top.setInverted(false);
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
    targetSpeedBot =
        (int) SmartDashboard.getNumber("Subsystems.Shooter.targetSpeedBot", targetSpeedBot);
    SmartDashboard.putNumber("Subsystems.Shooter.targetSpeedBot", targetSpeedBot);

    SmartDashboard.putBoolean("Subsystems.Shooter.isShooterReady", this.isShooterReady());

    SmartDashboard.putNumber("Subsystems.Shooter.yAngleAdjusted", this.getVisionYAngle());
    SmartDashboard.putNumber("Subsystems.Shooter.knotDistance", this.getKnotDistance());
  }

  public boolean isShooterReady() {
    return Math.abs(shooter_top.getVelocityError()) <= 600.0
        && Math.abs(shooter_bottom.getVelocityError()) <= 300.0;
  }

  public void setBottomMotor(double speed) {
    this.shooter_bottom.set(Math.abs(speed));
  }

  public void setTopMotor(double speed) {
    this.shooter_top.set(Math.abs(speed));
  }

  public double getShooterPowerTop() {
    return shooterPowerTop;
  }

  public double getShooterPowerBot() {
    return shooterPowerBot;
  }

  public void stop() {
    shooter_bottom.set(0);
    shooter_top.set(0);
  }

  public double getShooterSpeedTop() {
    return shooter_top.getSelectedSensorVelocity();
  }

  public double getShooterSpeedBot() {
    return shooter_bottom.getSelectedSensorVelocity();
  }

  public void setShooterVelTop(int speed) {
    speed = (int) MathUtil.clamp(speed, 0, maxRotationSpeedTop);
    shooter_top.configureWithPidParameters(topPidParameters, 0);
    this.shooter_top.set(ControlMode.Velocity, speed);
  }

  public void setShooterVelBot(int speed) {
    speed = (int) MathUtil.clamp(speed, 0, maxRotationSpeedBot);
    shooter_bottom.configureWithPidParameters(botPidParameters, 0);
    this.shooter_bottom.set(ControlMode.Velocity, speed);
  }

  public int getMaxVelTop() {
    return (int) maxRotationSpeedTop;
  }

  public int getMaxVelBot() {
    return (int) maxRotationSpeedBot;
  }

  public void setMotorVelocities(double topSpeedFraction, double botSpeedFraction) {
    setShooterVelTop((int) (topSpeedFraction * maxRotationSpeedTop));
    setShooterVelBot((int) (botSpeedFraction * maxRotationSpeedBot));
  }

  /*
  public int getTargetSpeedTop(){
    return targetSpeedTop;
  }
  */

  public int getTargetSpeedBot() {
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
