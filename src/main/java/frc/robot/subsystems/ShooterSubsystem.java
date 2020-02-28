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
import frc.robot.Constants;
import frc.robot.PidParameters;
import frc.robot.TeamTalonSRX;

public class ShooterSubsystem extends SubsystemBase {
  private double maxTopRotationSpeed=10000, maxBottomRotationSpeed=10000;
  private final TeamTalonSRX shooter_bottom;
  private final TeamTalonSRX shooter_top;
  /**
   * Creates a new ShooterSubsystem.
   */
  private double shooterPowerTop = -0.8;
  private double shooterPowerBot = -0.8;

  private PidParameters topPidParameters = new PidParameters(0,0,0,0,0,1,10);
  private PidParameters botPidParameters = new PidParameters(0,0,0,0,0,1,10);

  public ShooterSubsystem() {
    shooter_bottom = new TeamTalonSRX("Subsystems.Shooter.Bottom", Constants.BotShooterMotorCAN);
    shooter_top = new TeamTalonSRX("Subsystems.Shooter.Top", Constants.TopShooterMotorCAN);
  }

  @Override
  public void periodic() {
    shooter_bottom.periodic();
    shooter_top.periodic();
    topPidParameters.periodic("Subsystems.Shooter.Top", shooter_top, 0);
    botPidParameters.periodic("Subsystems.Shooter.Bot", shooter_bottom, 0);

    // This method will be called once per scheduler run
    shooterPowerTop = -Math.abs(SmartDashboard.getNumber("Subsystems.Shooter.shooterPowerTop", shooterPowerTop));
    SmartDashboard.putNumber("Subsystems.Shooter.shooterPowerTop", shooterPowerTop);
    shooterPowerBot = -Math.abs(SmartDashboard.getNumber("Subsystems.Shooter.shooterPowerBot", shooterPowerBot));
    SmartDashboard.putNumber("Subsystems.Shooter.shooterPowerBot", shooterPowerBot);

    maxTopRotationSpeed = SmartDashboard.getNumber("Subsystems.Shooter.maxTopotationSpeed", maxTopRotationSpeed);
    SmartDashboard.putNumber("Subsystems.Shooter.maxTopotationSpeed", maxTopRotationSpeed);
    maxBottomRotationSpeed = SmartDashboard.getNumber("Subsystems.Shooter.maxBottomotationSpeed", maxBottomRotationSpeed);
    SmartDashboard.putNumber("Subsystems.Shooter.maxBottomRotationSpeed", maxBottomRotationSpeed);
  }

  public void setBottomMotor(double speed){
    this.shooter_bottom.set(speed);
  }
  public void setTopMotor(double speed){
    this.shooter_top.set(speed);
  }
  public double getShooterPowerTop(){
    return shooterPowerTop;
  }
  public double getShooterPowerBot(){
    return shooterPowerBot;
  }
  public void stop(){
    shooter_bottom.set(0);
    shooter_top.set(0);
  }
  public int getShooterSpeedTop(){
    return shooter_top.getSelectedSensorVelocity();
  }
  public int getShooterSpeedBot(){
    return shooter_bottom.getSelectedSensorVelocity();
  }
  public void setShooterVelTop(int speed){
    this.shooter_top.set(ControlMode.Velocity, speed);
  }
  public void setShooterVelBot(int speed){
    this.shooter_top.set(ControlMode.Velocity, speed);
  }

  public void setMotorVelocities(double topSpeedFraction, double botSpeedFraction) {
    shooter_top.configureWithPidParameters(topPidParameters, 0);
    shooter_bottom.configureWithPidParameters(botPidParameters, 0);

    shooter_top.set(ControlMode.Velocity, topSpeedFraction*maxTopRotationSpeed);
    shooter_bottom.set(ControlMode.Velocity, botSpeedFraction*maxBottomRotationSpeed);
  }
}
